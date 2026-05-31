// Stash the rendered SVGs as an orphan commit kept alive by a CUSTOM ref
// (`refs/bench-renders/<run_id>`), NOT a branch. Refs outside refs/heads/* and
// refs/tags/* don't appear in GitHub's Branches or Tags UI, so per-run pushes
// don't pollute it — but the commit is still reachable, so it won't be GC'd and
// its blobs stay viewable in the web UI. Links use the commit SHA directly
// (GitHub's blob viewer resolves any SHA reachable from any ref), so we never
// need a branch/tag name in the URL.
//
// We do NOT create one blob per SVG. GitHub's *secondary* rate limit throttles
// bursts of content-creating writes (~900 points/min, 5 points per mutating
// POST); at 700+ trajectories x2 sides a per-file `createBlob` loop fires ~1.5k
// POSTs back-to-back and trips it, 60s-blocking the rest of the job (including
// the report publish that runs right after). Instead the Git Trees API creates
// the blobs inline — each tree entry carries `content` directly — so the whole
// publish is a handful of `createTree` calls + one commit + one ref.

const fs = require("fs");
const path = require("path");

// Entries per createTree request. At our ~46 KB-mean SVGs this keeps each
// request body around ~5 MB, well under GitHub's payload ceiling, while
// collapsing ~1.5k blob POSTs into ~15 tree calls.
const CHUNK = 100;

module.exports = async ({ github, context, core }) => {
  const { owner, repo } = context.repo;
  const runId = process.env.GITHUB_RUN_ID;
  const refName = `bench-renders/${runId}`;

  const entries = [];
  for (const root of ["merged/tp-pr", "merged/tp-base"]) {
    if (!fs.existsSync(root)) continue;
    for (const rel of fs.readdirSync(root, { recursive: true })) {
      if (!rel.endsWith(".svg")) continue;
      const abs = path.join(root, rel);
      if (!fs.statSync(abs).isFile()) continue;
      entries.push({
        path: rel.split(path.sep).join("/"),
        mode: "100644",
        type: "blob",
        content: fs.readFileSync(abs, "utf8"), // inline — no separate createBlob
      });
    }
  }
  if (entries.length === 0) {
    core.info("no SVGs to publish");
    return;
  }

  // Build the tree in chunks, chaining each onto the previous via base_tree so a
  // single huge request body never hits GitHub's payload ceiling. base_tree only
  // affects tree construction, not commit ancestry — the commit below is still
  // an orphan (parents: []).
  let baseTree;
  for (let i = 0; i < entries.length; i += CHUNK) {
    const t = await github.rest.git.createTree({
      owner,
      repo,
      tree: entries.slice(i, i + CHUNK),
      ...(baseTree ? { base_tree: baseTree } : {}),
    });
    baseTree = t.data.sha;
  }

  const c = await github.rest.git.createCommit({
    owner,
    repo,
    tree: baseTree,
    parents: [],
    message: `bench renders — run ${runId} (${(process.env.HEAD_SHA || "").slice(0, 7)})`,
  });
  await github.rest.git.createRef({
    owner,
    repo,
    ref: `refs/${refName}`,
    sha: c.data.sha,
  });
  core.setOutput(
    "url",
    `${process.env.GITHUB_SERVER_URL}/${owner}/${repo}/blob/${c.data.sha}`,
  );
};
