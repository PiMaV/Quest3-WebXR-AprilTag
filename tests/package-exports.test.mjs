import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { fileURLToPath, pathToFileURL } from 'node:url';

const rootDir = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const pkg = JSON.parse(fs.readFileSync(path.join(rootDir, 'package.json'), 'utf8'));

function extractLocalSpecifiers(sourceText) {
  const specs = [];
  const re = /(?:import|export)\s*(?:[^'"]*?\sfrom\s*)?['"](\.[^'"]+)['"]/g;
  for (const m of sourceText.matchAll(re)) specs.push(m[1]);
  return specs;
}

function collectExportedTypeNames(dtsText) {
  const names = new Set();
  const declRe = /^export\s+(?:declare\s+)?(?:function|const|let|var|class|interface|type|enum)\s+([A-Za-z_$][\w$]*)/gm;
  for (const m of dtsText.matchAll(declRe)) names.add(m[1]);

  const listRe = /^export\s*\{([^}]+)\}(?:\s*from\s*['"][^'"]+['"])?\s*;?/gm;
  for (const m of dtsText.matchAll(listRe)) {
    for (const part of m[1].split(',')) {
      const cleaned = part.replace(/^\s*type\s+/, '').trim();
      if (!cleaned) continue;
      const asMatch = cleaned.match(/\bas\s+([A-Za-z_$][\w$]*)$/);
      names.add(asMatch ? asMatch[1] : cleaned);
    }
  }
  return names;
}

function stripCommentsAndStrings(src) {
  return src
    .replace(/\/\*[\s\S]*?\*\//g, '')
    .replace(/\/\/.*$/gm, '')
    .replace(/'(?:\\.|[^'\\])*'|"(?:\\.|[^"\\])*"/g, '""');
}

function assertBalancedDelimiters(text, label) {
  const cleaned = stripCommentsAndStrings(text);
  const stack = [];
  const pairs = { '(': ')', '[': ']', '{': '}' };
  const close = new Set([')', ']', '}']);
  for (const ch of cleaned) {
    if (pairs[ch]) stack.push(pairs[ch]);
    else if (close.has(ch)) {
      assert.equal(stack.pop(), ch, `unbalanced delimiter ${ch} in ${label}`);
    }
  }
  assert.equal(stack.length, 0, `unclosed delimiters in ${label}`);
}

test('src/index.mjs import graph has zero Three.js dependency', () => {
  const visited = new Set();
  const queue = [path.join(rootDir, 'src', 'index.mjs')];
  while (queue.length) {
    const file = queue.shift();
    if (visited.has(file)) continue;
    visited.add(file);
    const rel = path.relative(rootDir, file).replace(/\\/g, '/');
    assert.ok(!rel.startsWith('vendor/three/'), `src/index.mjs must not transitively import ${rel}`);
    assert.notEqual(rel, 'src/render-batch.mjs', 'src/index.mjs must not import src/render-batch.mjs');
    assert.notEqual(rel, 'src/ui-atlas.mjs', 'src/index.mjs must not import src/ui-atlas.mjs');
    const txt = fs.readFileSync(file, 'utf8');
    for (const spec of extractLocalSpecifiers(txt)) {
      const target = path.resolve(path.dirname(file), spec);
      if (fs.existsSync(target) && fs.statSync(target).isFile()) queue.push(target);
    }
  }
  assert.ok(visited.size >= 15);
});

test('src/three.mjs exports all 7 optional Three.js helpers', async () => {
  const threeMod = await import('../src/three.mjs');
  const keys = Object.keys(threeMod).sort();
  assert.deepEqual(keys, [
    'createAxesBatch',
    'createBadgeAtlas',
    'createBillboardAtlasMesh',
    'createInstancedPool',
    'createOrientedAtlasMesh',
    'createPathRibbonMesh',
    'createPathTubeMesh',
  ]);
});

test('package.json exports resolve to disk files and import cleanly in Node', async () => {
  assert.equal(typeof pkg.types, 'string');
  assert.ok(fs.existsSync(path.join(rootDir, pkg.types)), `missing pkg.types: ${pkg.types}`);

  for (const [subpath, entry] of Object.entries(pkg.exports)) {
    if (subpath.includes('*')) continue;
    const importPath = typeof entry === 'string' ? entry : entry.import;
    const typesPath = typeof entry === 'object' ? entry.types : null;

    assert.ok(importPath, `missing import path for ${subpath}`);
    const absImport = path.join(rootDir, importPath);
    assert.ok(fs.existsSync(absImport), `missing file for export ${subpath}: ${importPath}`);

    if (typesPath) {
      const absTypes = path.join(rootDir, typesPath);
      assert.ok(fs.existsSync(absTypes), `missing types file for export ${subpath}: ${typesPath}`);
    }

    if (importPath.endsWith('.mjs') && !importPath.endsWith('-worker.mjs')) {
      const mod = await import(pathToFileURL(absImport).href);
      assert.ok(Object.keys(mod).length > 0, `${importPath} must export at least one symbol`);
    }
  }
});

test('TypeScript declaration files match runtime exports and have valid delimiter structure', async () => {
  const dtsFiles = fs.readdirSync(path.join(rootDir, 'types')).filter((f) => f.endsWith('.d.ts'));
  assert.ok(dtsFiles.length >= 14);
  for (const file of dtsFiles) {
    const content = fs.readFileSync(path.join(rootDir, 'types', file), 'utf8');
    assertBalancedDelimiters(content, `types/${file}`);
  }

  for (const [subpath, entry] of Object.entries(pkg.exports)) {
    if (typeof entry !== 'object' || !entry.import || !entry.types) continue;
    const runtimeMod = await import(pathToFileURL(path.join(rootDir, entry.import)).href);
    const dtsContent = fs.readFileSync(path.join(rootDir, entry.types), 'utf8');
    const declared = collectExportedTypeNames(dtsContent);
    for (const exportName of Object.keys(runtimeMod)) {
      assert.ok(
        declared.has(exportName),
        `Export ${exportName} from ${entry.import} (subpath ${subpath}) missing in ${entry.types}`,
      );
    }
  }
});

test('examples/minimal.mjs and examples/quest.mjs pass syntax check and only import existing exports', async () => {
  const coreMod = await import('../src/index.mjs');
  const threeMod = await import('../src/three.mjs');
  const modMap = new Map([
    ['../src/index.mjs', coreMod],
    ['../src/three.mjs', threeMod],
  ]);

  for (const relPath of ['examples/minimal.mjs', 'examples/quest.mjs']) {
    const absPath = path.join(rootDir, relPath);
    execFileSync(process.execPath, ['--check', absPath], { stdio: 'pipe' });
    const text = fs.readFileSync(absPath, 'utf8');
    const namedImportRe = /import\s*\{([^}]+)\}\s*from\s*['"](\.\.\/src\/[^'"]+)['"]\s*;/g;
    for (const m of text.matchAll(namedImportRe)) {
      const targetSpec = m[2];
      const mod = modMap.get(targetSpec) ?? (await import(pathToFileURL(path.resolve(path.dirname(absPath), targetSpec)).href));
      for (const rawName of m[1].split(',')) {
        const cleaned = rawName.trim();
        if (!cleaned) continue;
        const orig = cleaned.split(/\s+as\s+/)[0].trim();
        assert.ok(orig in mod, `${relPath} imports non-existent symbol "${orig}" from ${targetSpec}`);
      }
    }
  }
});
