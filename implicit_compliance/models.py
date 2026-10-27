"""Small motor-history classifier and the course's two-head ResNet-50 baseline."""
import torch
from torch import nn


class JointStateCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.features = nn.Sequential(nn.Conv1d(9, 32, 7, padding=3), nn.BatchNorm1d(32), nn.SiLU(),
                                      nn.Conv1d(32, 64, 5, stride=2, padding=2), nn.BatchNorm1d(64), nn.SiLU(),
                                      nn.Conv1d(64, 64, 3, padding=1), nn.SiLU(), nn.AdaptiveAvgPool1d(1), nn.Flatten())
        self.contact = nn.Linear(64, 1)
        self.incline = nn.Linear(64, 6)
        self.material = nn.Linear(64, 6)

    def forward(self, x):
        h = self.features(x.transpose(1, 2))
        return self.contact(h).squeeze(-1), self.incline(h), self.material(h)


class ExplicitClassifier(nn.Module):
    def __init__(self, pretrained=True):
        super().__init__()
        from torchvision.models import resnet50, ResNet50_Weights
        self.backbone = resnet50(weights=ResNet50_Weights.DEFAULT if pretrained else None)
        width = self.backbone.fc.in_features
        self.backbone.fc = nn.Identity()
        self.incline = nn.Linear(width, 6)
        self.material = nn.Linear(width, 6)

    def forward(self, x):
        h = self.backbone(x)
        return self.incline(h), self.material(h)


class DeployContact(nn.Module):
    """All preprocessing except physical-time resampling travels with the model."""
    def __init__(self, model, mean, std):
        super().__init__()
        self.model = model
        self.register_buffer("mean", torch.as_tensor(mean, dtype=torch.float32))
        self.register_buffer("std", torch.as_tensor(std, dtype=torch.float32))

    def forward(self, raw_motor):
        contact, incline, material = self.model((raw_motor - self.mean) / self.std)
        return torch.sigmoid(contact), torch.softmax(incline, -1), torch.softmax(material, -1)


class PCGrad:
    """Project conflicting task gradients; average shared and sum private gradients."""
    def __init__(self, optimizer, seed=123):
        self.optimizer = optimizer
        self.generator = torch.Generator().manual_seed(seed)

    def step(self, losses):
        params = [p for group in self.optimizer.param_groups for p in group["params"] if p.requires_grad]
        grads, masks = [], []
        for i, loss in enumerate(losses):
            gs = torch.autograd.grad(loss, params, retain_graph=i < len(losses)-1, allow_unused=True)
            grads.append(torch.cat([torch.zeros_like(p).flatten() if g is None else g.flatten() for p, g in zip(params, gs)]))
            masks.append(torch.cat([torch.full_like(p, g is not None, dtype=torch.bool).flatten() for p, g in zip(params, gs)]))
        projected = []
        for original in grads:
            grad = original.clone()
            for j in torch.randperm(len(grads), generator=self.generator).tolist():
                other = grads[j]
                dot = torch.dot(grad, other)
                grad = grad - torch.minimum(dot, torch.zeros_like(dot)) * other / torch.dot(other, other).clamp_min(1e-12)
            projected.append(grad)
        shared = torch.stack(masks).all(dim=0)
        stacked = torch.stack(projected)
        merged = torch.where(shared, stacked.mean(dim=0), stacked.sum(dim=0))
        offset = 0
        self.optimizer.zero_grad(set_to_none=True)
        for p in params:
            n = p.numel()
            p.grad = merged[offset:offset+n].view_as(p).clone()
            offset += n
        self.optimizer.step()
