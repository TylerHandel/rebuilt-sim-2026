// Put files onto a side branch (batch-results, batch-queue) and push it, without touching the
// checkout, its index or its branch: a scratch index on the branch's tip from origin, the files
// added, a commit on top, pushed. Retries once on a race with another push.
import { execFileSync } from 'child_process';
import { readFileSync, rmSync } from 'fs';
import { tmpdir } from 'os';
import { join } from 'path';

export function git(cwd, args, extra = {}) {
  return execFileSync('git', args, { cwd, env: { ...process.env, ...extra }, stdio: ['ignore', 'pipe', 'pipe'] }).toString().trim();
}

// the branch's tip on origin ('' if it doesn't exist yet)
export function tip(cwd, branch) {
  try { git(cwd, ['fetch', '-q', 'origin', branch]); return git(cwd, ['rev-parse', 'FETCH_HEAD']); } catch { return ''; }
}

// files: { 'path/in/branch': 'local file path' or { text } }
export function pushFiles(cwd, branch, files, message) {
  const idx = { GIT_INDEX_FILE: join(tmpdir(), `sim-index-${process.pid}-${Date.now()}`) };
  const who = {};
  try { git(cwd, ['config', 'user.email']); } catch { Object.assign(who, { GIT_AUTHOR_NAME: 'batch', GIT_AUTHOR_EMAIL: 'batch@localhost', GIT_COMMITTER_NAME: 'batch', GIT_COMMITTER_EMAIL: 'batch@localhost' }); }
  for (let attempt = 0; ; attempt++) {
    try {
      const parent = tip(cwd, branch);
      if (parent) git(cwd, ['read-tree', parent], idx); else git(cwd, ['read-tree', '--empty'], idx);
      for (const [path, src] of Object.entries(files)) {
        const body = typeof src === 'string' ? readFileSync(src) : Buffer.from(src.text);
        const blob = execFileSync('git', ['hash-object', '-w', '--stdin'], { cwd, input: body }).toString().trim();
        git(cwd, ['update-index', '--add', '--cacheinfo', `100644,${blob},${path}`], idx);
      }
      const tree = git(cwd, ['write-tree'], idx);
      const c = git(cwd, ['commit-tree', tree, ...(parent ? ['-p', parent] : []), '-m', message], who);
      git(cwd, ['push', '-q', 'origin', `${c}:refs/heads/${branch}`]);
      return c;
    } catch (e) {
      if (attempt >= 2) throw e;
    } finally {
      rmSync(idx.GIT_INDEX_FILE, { force: true });
    }
  }
}
