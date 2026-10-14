'use strict';

// Resolve the branch a stacked pull request eventually merges into. Used by the
// integration image resolver in .github/workflows/ci.yaml, which needs the
// branch the stack lands on rather than the head branch of the PR below.

const RELEASE_OR_MAIN = /^(main|v\d+\.\d+(\.\d+)?)$/;

/**
 * Follow open PRs down the stack: while the base is the head of an open PR,
 * move to that PR's base. The first branch that heads no open PR is where the
 * stack lands. The walk stops at `main`, `vX.Y`, and `vX.Y.Z` without a lookup,
 * so an open forward-merge PR cannot carry a PR on `v10.1` over to `v10.2`.
 *
 * Returns `{ base, chain, resolved }`. `resolved` is false when the walk stopped
 * at a branch that heads several open PRs, or at a cycle. `chain` reads like
 * `#3 -> fix/b -> #2 -> v10.2` and names where it stopped.
 */
async function resolveStackBase({ github, owner, repo, pullRequest }) {
  let base = pullRequest.base.ref;
  const chain = [`#${pullRequest.number}`, base];
  const visited = new Set();
  let open;
  while (!RELEASE_OR_MAIN.test(base) && !visited.has(base)) {
    visited.add(base);
    // example_ws is a GitHub fork, and in a fork network pulls.list's `head`
    // filter returns nothing, so list the open PRs once and match heads here.
    open ??= await github.paginate(github.rest.pulls.list, { owner, repo, state: 'open', per_page: 100 });
    const below = open.filter((pr) => pr.head.ref === base && pr.head.repo?.full_name === `${owner}/${repo}`);
    if (below.length === 0) {
      return { base, chain: chain.join(' -> '), resolved: true };
    }
    if (below.length > 1) {
      chain.push('(head of several open PRs)');
      return { base, chain: chain.join(' -> '), resolved: false };
    }
    base = below[0].base.ref;
    chain.push(`#${below[0].number}`, base);
  }
  return { base, chain: chain.join(' -> '), resolved: !visited.has(base) };
}

module.exports = { RELEASE_OR_MAIN, resolveStackBase };
