// Publish the full report.md as an orphan commit on a CUSTOM ref
// (`refs/bench-report/<run_id>`), NOT a branch — same hidden-ref trick as
// publish-renders.cjs, so per-run pushes never show in the Branches/Tags UI but
// the blob stays viewable in GitHub's web UI. The slim PR comment links here for
// the per-trajectory detail it omits (GitHub renders the .md blob in-browser).

const fs = require("fs");

module.exports = async ({ github, context, core }) => {
  const { owner, repo } = context.repo;
  const runId = process.env.GITHUB_RUN_ID;
  const reportPath = process.env.REPORT_MD || "report.md";

  if (!fs.existsSync(reportPath)) {
    core.info(`no report at ${reportPath} to publish`);
    return;
  }

  const blob = await github.rest.git.createBlob({
    owner,
    repo,
    content: fs.readFileSync(reportPath).toString("base64"),
    encoding: "base64",
  });
  const tree = await github.rest.git.createTree({
    owner,
    repo,
    tree: [{ path: "report.md", mode: "100644", type: "blob", sha: blob.data.sha }],
  });
  const c = await github.rest.git.createCommit({
    owner,
    repo,
    tree: tree.data.sha,
    parents: [],
    message: `bench report — run ${runId} (${(process.env.HEAD_SHA || "").slice(0, 7)})`,
  });
  await github.rest.git.createRef({
    owner,
    repo,
    ref: `refs/bench-report/${runId}`,
    sha: c.data.sha,
  });
  core.setOutput(
    "url",
    `${process.env.GITHUB_SERVER_URL}/${owner}/${repo}/blob/${c.data.sha}/report.md`,
  );
};
