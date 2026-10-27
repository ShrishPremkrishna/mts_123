"""Train/evaluate/export without allowing held-out recordings into model selection."""
from __future__ import annotations
import argparse
import copy
import json
import random
import subprocess
import time
from pathlib import Path
import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset
from .data import sha256
from .models import JointStateCNN, ExplicitClassifier, DeployContact, PCGrad


class MotorDataset(Dataset):
    def __init__(self, manifest_path, split, max_samples=None):
        self.path = Path(manifest_path)
        self.manifest = json.loads(self.path.read_text())
        self.records, self.index = [], []
        self.mean = np.asarray(self.manifest["normalization"]["mean"], dtype=np.float32)
        self.std = np.asarray(self.manifest["normalization"]["std"], dtype=np.float32)
        self.window = self.manifest["window"]
        for record in self.manifest["records"]:
            if record["split"] != split:
                continue
            with np.load(self.path.parent / record["cache"], allow_pickle=False) as f:
                data = dict(motor=f["motor"], contact=f["contact"], starts=f["starts"])
            i = len(self.records)
            self.records.append((record, data))
            self.index.extend((i, int(s)) for s in data["starts"])
        if max_samples is not None and len(self.index) > max_samples:
            chosen = np.random.default_rng(self.manifest["seed"]).choice(len(self.index), max_samples, replace=False)
            self.index = [self.index[i] for i in sorted(chosen)]

    def __len__(self):
        return len(self.index)

    def __getitem__(self, index):
        i, start = self.index[index]
        record, data = self.records[i]
        x = (data["motor"][start:start+self.window] - self.mean) / self.std
        return torch.from_numpy(x), torch.tensor([data["contact"][start+self.window-1], record["incline"], record["material"]], dtype=torch.long)


class ImageDataset(Dataset):
    def __init__(self, manifest_path, split, root=None, max_samples=None):
        from torchvision import transforms
        from PIL import Image
        self.Image = Image
        self.manifest = json.loads(Path(manifest_path).read_text())
        self.root = Path(root or self.manifest["root"])
        self.samples = [s for s in self.manifest["images"] if s["split"] == split]
        if max_samples is not None and len(self.samples) > max_samples:
            chosen = np.random.default_rng(self.manifest["seed"]).choice(len(self.samples), max_samples, replace=False)
            self.samples = [self.samples[i] for i in sorted(chosen)]
        # No color augmentation: material/lighting ablation must remain interpretable.
        self.transform = transforms.Compose([transforms.Resize(256), transforms.CenterCrop(224), transforms.ToTensor(),
                                              transforms.Normalize([.485,.456,.406],[.229,.224,.225])])

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, i):
        item = self.samples[i]
        with self.Image.open(self.root / item["path"]) as im:
            x = self.transform(im.convert("RGB"))
        return x, torch.tensor([1, item["incline"], item["material"]], dtype=torch.long)


def classification_metrics(truth, predicted, classes):
    confusion = np.zeros((classes, classes), dtype=int)
    for a, b in zip(truth, predicted):
        confusion[int(a), int(b)] += 1
    support = confusion.sum(axis=1)
    recall = np.divide(np.diag(confusion), support, out=np.zeros(classes, dtype=float), where=support > 0)
    observed = support > 0
    result = dict(samples=int(support.sum()), accuracy=float(np.trace(confusion)/support.sum()) if support.sum() else None,
                  balanced_accuracy=float(recall[observed].mean()) if observed.any() else None,
                  recall=[float(r) if n else None for r, n in zip(recall, support)], support=support.tolist(), confusion=confusion.tolist())
    if classes == 2:
        tp, fp, fn = confusion[1,1], confusion[0,1], confusion[1,0]
        result.update(precision=float(tp/(tp+fp)) if tp+fp else None, sensitivity=float(tp/(tp+fn)) if tp+fn else None,
                      false_positive_rate=float(fp/support[0]) if support[0] else None,
                      f1=float(2*tp/(2*tp+fp+fn)) if 2*tp+fp+fn else None)
    return result


@torch.inference_mode()
def evaluate(model, loader, device, kind):
    model.eval()
    truth = [[],[],[]]
    predicted = [[],[],[]]
    deploy_contact = []
    for x, y in loader:
        output = model(x.to(device))
        yc = y.numpy()
        if kind == "implicit":
            contact, incline, material = output
            truth[0].extend(yc[:,0]); predicted[0].extend((contact > 0).cpu().numpy())
            deploy_contact.extend((contact >= np.log(.8/.2)).cpu().numpy())
        else:
            incline, material = output
        mask = yc[:,0].astype(bool)
        for j, logits in ((1,incline),(2,material)):
            truth[j].extend(yc[mask,j]); predicted[j].extend(logits.argmax(-1).cpu().numpy()[mask])
    result = {name: classification_metrics(truth[i], predicted[i], 2 if i == 0 else 6)
              for i, name in enumerate(("contact", "incline", "material")) if kind == "implicit" or i > 0}
    if kind == "implicit":
        result["contact"]["at_streaming_on_threshold_0_8"] = classification_metrics(truth[0],deploy_contact,2)
    return result


def task_losses(output, y, kind, contact_weight=1.):
    if kind == "implicit":
        contact, incline, material = output
        losses = [torch.nn.functional.binary_cross_entropy_with_logits(contact, y[:,0].float()) * contact_weight]
    else:
        incline, material = output
        losses = []
    mask = y[:,0].bool()
    if mask.any():
        # Mean over contacts only; changing contact prevalence must not silently rescale the terrain tasks.
        losses.extend([torch.nn.functional.cross_entropy(incline[mask], y[mask,1]),
                       torch.nn.functional.cross_entropy(material[mask], y[mask,2])])
    return losses


def choose_device(request):
    if request != "auto":
        return torch.device(request)
    return torch.device("cuda" if torch.cuda.is_available() else "mps" if torch.backends.mps.is_available() else "cpu")


def load_checkpoint(path, device):
    ckpt = torch.load(path, map_location="cpu", weights_only=True)
    model = JointStateCNN() if ckpt["kind"] == "implicit" else ExplicitClassifier(pretrained=False)
    model.load_state_dict(ckpt["state_dict"])
    return model.to(device), ckpt


def make_dataset(args, split):
    if args.kind == "implicit":
        return MotorDataset(args.manifest, split, args.max_samples)
    return ImageDataset(args.manifest, split, args.root, args.max_samples)


def fit(args):
    out = Path(args.out)
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f"Use a new run directory: {out}")
    out.mkdir(parents=True, exist_ok=True)
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    torch.set_num_threads(args.threads)
    device = choose_device(args.device)
    train_data, val_data = make_dataset(args, "train"), make_dataset(args, "val")
    if not len(train_data) or not len(val_data):
        raise ValueError("Training and validation data are both required")
    gen = torch.Generator().manual_seed(args.seed)
    train_loader = DataLoader(train_data, args.batch_size, shuffle=True, generator=gen, num_workers=args.workers)
    val_loader = DataLoader(val_data, args.batch_size, shuffle=False, num_workers=args.workers)
    model = JointStateCNN() if args.kind == "implicit" else ExplicitClassifier(pretrained=not args.no_pretrained)
    model.to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    pcgrad = PCGrad(optimizer, args.seed)
    manifest = json.loads(Path(args.manifest).read_text())
    try:
        revision = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        revision = None
    meta = dict(kind=args.kind, manifest_sha256=sha256(args.manifest), seed=args.seed, device=str(device),
                train_samples=len(train_data), validation_samples=len(val_data), git_revision=revision,
                torch_version=str(torch.__version__), optimizer="PCGrad AdamW" if args.pcgrad else "summed-loss AdamW",
                args={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
                limited_smoke_data=args.max_samples is not None,
                preprocessing={k:manifest[k] for k in ("channels","inclines","materials","window","sample_hz","max_gap_s","normalization","label_rule")})
    (out/"run.json").write_text(json.dumps(meta,indent=2)+"\n")
    best = -float("inf")
    history = []
    start = time.monotonic()
    for epoch in range(1,args.epochs+1):
        model.train()
        weight = 1. - .9*(epoch-1)/max(1,args.epochs-1)
        total_loss, seen = 0., 0
        for x,y in train_loader:
            x,y = x.to(device),y.to(device)
            output = model(x)
            losses = task_losses(output,y,args.kind,weight)
            value = sum(losses)
            if args.pcgrad:
                pcgrad.step(losses)
            else:
                optimizer.zero_grad(set_to_none=True); value.backward(); optimizer.step()
            total_loss += float(value.detach())*len(x); seen += len(x)
        metrics = evaluate(model,val_loader,device,args.kind)
        score = np.mean([m["balanced_accuracy"] for m in metrics.values() if m["balanced_accuracy"] is not None])
        row = dict(epoch=epoch,train_loss=total_loss/seen,validation=metrics,selection_score=float(score),elapsed_s=time.monotonic()-start)
        history.append(row)
        (out/"history.json").write_text(json.dumps(history,indent=2)+"\n")
        state = {**meta,"epoch":epoch,"state_dict":{k:v.detach().cpu() for k,v in model.state_dict().items()},"validation":metrics}
        torch.save(state,out/"last.pt")
        if score > best:
            best=float(score); torch.save(state,out/"best.pt")
        print(json.dumps(row),flush=True)
    # Final report intentionally excludes test; use evaluate once model selection is done.
    return out/"best.pt"


def eval_checkpoint(args):
    device=choose_device(args.device)
    model,ckpt=load_checkpoint(args.checkpoint,device)
    if sha256(args.manifest)!=ckpt["manifest_sha256"]:
        raise ValueError("Manifest differs from training; use the original frozen split")
    args.kind=ckpt["kind"]
    dataset=make_dataset(args,args.split)
    if not len(dataset):
        raise ValueError("Requested split is empty")
    metrics=evaluate(model,DataLoader(dataset,args.batch_size,shuffle=False,num_workers=args.workers),device,args.kind)
    report=dict(checkpoint_sha256=sha256(args.checkpoint),manifest_sha256=sha256(args.manifest),split=args.split,metrics=metrics,
                scope="recording/condition-held-out course dataset; not a robot deployment result",limited_smoke_data=args.max_samples is not None)
    Path(args.out).write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))


def export(args):
    model,ckpt=load_checkpoint(args.checkpoint,torch.device("cpu"))
    if ckpt["kind"]!="implicit":
        raise ValueError("Robot streaming export is only for the motor classifier")
    metadata=ckpt["preprocessing"]
    wrapper=DeployContact(model.eval(),metadata["normalization"]["mean"],metadata["normalization"]["std"]).eval()
    scripted=torch.jit.script(wrapper)
    # Export parity uses nonzero motor values, checking embedded normalization too.
    raw=torch.randn(3,metadata["window"],9)
    with torch.no_grad():
        for a,b in zip(wrapper(raw),scripted(raw)):
            torch.testing.assert_close(a,b)
    output=Path(args.out); output.mkdir(parents=True,exist_ok=True)
    scripted.save(str(output/"contact.pt"))
    torch.set_num_threads(1)
    sample=raw[:1]
    times=[]
    with torch.inference_mode():
        for i in range(110):
            begin=time.perf_counter(); scripted(sample)
            if i>=10: times.append((time.perf_counter()-begin)*1000.)
    benchmark=dict(device="cpu",threads=1,batch_size=1,samples=len(times),
                   median_ms=float(np.median(times)),p95_ms=float(np.percentile(times,95)),max_ms=max(times),
                   scope="this development host, not Raspberry Pi",model_bytes=(output/"contact.pt").stat().st_size)
    (output/"benchmark.json").write_text(json.dumps(benchmark,indent=2)+"\n")
    metadata={**metadata,"model_sha256":sha256(output/"contact.pt"),"checkpoint_sha256":sha256(args.checkpoint),
              "outputs":["contact_probability","incline_probabilities","material_probabilities"],
              "status":"trained on course dataset; hardware validation required"}
    (output/"metadata.json").write_text(json.dumps(metadata,indent=2)+"\n")
    print(output)


def main():
    p=argparse.ArgumentParser(description=__doc__); sub=p.add_subparsers(dest="command",required=True)
    train=sub.add_parser("train"); ev=sub.add_parser("evaluate"); ex=sub.add_parser("export")
    for parser in (train,ev):
        parser.add_argument("--manifest",type=Path,required=True); parser.add_argument("--out",type=Path,required=True)
        parser.add_argument("--root",type=Path); parser.add_argument("--device",default="auto")
        parser.add_argument("--batch-size",type=int,default=128); parser.add_argument("--workers",type=int,default=0)
        parser.add_argument("--max-samples",type=int,help="Smoke tests only: marks all outputs limited")
    train.add_argument("--kind",choices=["implicit","explicit"],default="implicit")
    train.add_argument("--epochs",type=int,default=50); train.add_argument("--lr",type=float,default=.001)
    train.add_argument("--seed",type=int,default=123); train.add_argument("--threads",type=int,default=4)
    train.add_argument("--no-pretrained",action="store_true"); train.add_argument("--pcgrad",action=argparse.BooleanOptionalAction,default=True)
    ev.add_argument("--checkpoint",type=Path,required=True); ev.add_argument("--split",choices=["val","test"],default="test")
    ex.add_argument("--checkpoint",type=Path,required=True); ex.add_argument("--out",type=Path,required=True)
    args=p.parse_args()
    if args.command=="train":
        if args.epochs<1: p.error("epochs must be positive")
        fit(args)
    elif args.command=="evaluate": eval_checkpoint(args)
    else: export(args)

if __name__=="__main__": main()
