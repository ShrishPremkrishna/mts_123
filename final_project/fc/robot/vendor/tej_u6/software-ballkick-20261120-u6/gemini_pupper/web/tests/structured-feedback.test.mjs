import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { build } from 'esbuild';
const bundle = await build({entryPoints:['services/operatorFeedback.ts'],bundle:true,write:false,format:'esm',platform:'node'});
const {operatorFeedback}=await import('data:text/javascript;base64,'+Buffer.from(bundle.outputFiles[0].text).toString('base64'));
const cases=JSON.parse(fs.readFileSync(new URL('../../tests/fixtures/structured-feedback.json',import.meta.url),'utf8'));
for(const c of cases)test(c.title,()=>{const text=operatorFeedback(c.name,c.result);assert.equal(text,c.expected);assert.ok(text.length<=240);assert.ok(!text.includes('INJECTED'));});
