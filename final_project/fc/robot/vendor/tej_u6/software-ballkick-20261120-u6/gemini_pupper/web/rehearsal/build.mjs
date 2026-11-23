// Isolated build only. The production Vite entry never imports this rehearsal.
import { build } from 'esbuild';
import { mkdir, readFile, writeFile, copyFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { resolve } from 'node:path';
import { createHash } from 'node:crypto';
const web = fileURLToPath(new URL('../', import.meta.url));
const out = resolve(process.argv[2] || '/private/tmp/pupper-browser-media-rehearsal');
await mkdir(out, { recursive: true });
const result = await build({ absWorkingDir: web, entryPoints: ['rehearsal/main.ts'], bundle: true,
  outfile: `${out}/rehearsal.js`, platform: 'browser', format: 'esm', metafile: true,
  alias: { '@google/genai': resolve(web, 'rehearsal/mockGoogle.ts') } });
await copyFile(resolve(web, 'rehearsal/index.html'), `${out}/index.html`);
const digest = bytes => createHash('sha256').update(bytes).digest('hex');
const inputs = {};
for (const path of [...Object.keys(result.metafile.inputs), 'rehearsal/index.html', 'rehearsal/build.mjs']) inputs[path] = digest(await readFile(resolve(web, path)));
const outputs = {};
for (const path of ['rehearsal.js', 'index.html']) outputs[path] = digest(await readFile(`${out}/${path}`));
await writeFile(`${out}/build-receipt.json`, JSON.stringify({ mode: 'isolated fake-only media rehearsal', inputs, outputs }, null, 2) + '\n');
console.log(JSON.stringify({ out, outputs }));
