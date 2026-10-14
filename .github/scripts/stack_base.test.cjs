'use strict';

const assert = require('node:assert/strict');
const test = require('node:test');

const { resolveStackBase } = require('./stack_base.cjs');

const REPO = 'PickNikRobotics/moveit_pro_example_ws';

// A fake `github` whose open PRs are `{ head: [[number, base, headRepo?], ...] }`.
// Like GitHub on this fork, it ignores `head` and lists every open PR.
function fakeGithub(openPrs) {
  const calls = [];
  const list = async (params) => {
    calls.push(params);
    const data = Object.entries(openPrs).flatMap(([ref, prs]) =>
      prs.map(([number, base, headRepo = REPO]) => ({
        number,
        head: { ref, repo: { full_name: headRepo } },
        base: { ref: base },
      }))
    );
    return { data };
  };
  return {
    calls,
    paginate: async (method, params) => (await method(params)).data,
    rest: { pulls: { list } },
  };
}

function resolve(base, openPrs = {}) {
  const github = fakeGithub(openPrs);
  const result = resolveStackBase({
    github,
    owner: 'PickNikRobotics',
    repo: 'moveit_pro_example_ws',
    pullRequest: { number: 9, base: { ref: base } },
  });
  return result.then((r) => ({ ...r, calls: github.calls }));
}

test('a PR on main or a release branch resolves without an API call', async () => {
  for (const base of ['main', 'v10.2', 'v10.1.0']) {
    const r = await resolve(base);
    assert.deepEqual(
      { base: r.base, chain: r.chain, resolved: r.resolved, calls: r.calls.length },
      { base, chain: `#9 -> ${base}`, resolved: true, calls: 0 }
    );
  }
});

test('a stack resolves to the base of its bottom PR', async () => {
  const r = await resolve('fix/c', { 'fix/c': [[3, 'fix/b']], 'fix/b': [[2, 'v10.2']] });
  assert.equal(r.base, 'v10.2');
  assert.equal(r.resolved, true);
  assert.equal(r.chain, '#9 -> fix/c -> #3 -> fix/b -> #2 -> v10.2');
  assert.deepEqual(r.calls, [
    { owner: 'PickNikRobotics', repo: 'moveit_pro_example_ws', state: 'open', per_page: 100 },
  ]);
});

test('a same-named branch on a fork is not a parent', async () => {
  const fork = 'someone/moveit_pro_example_ws';
  const onlyFork = await resolve('fix/a', { 'fix/a': [[1, 'main', fork]] });
  assert.equal(onlyFork.base, 'fix/a');
  assert.equal(onlyFork.resolved, true);

  const both = await resolve('fix/a', { 'fix/a': [[1, 'v10.2'], [2, 'main', fork]] });
  assert.equal(both.base, 'v10.2');
  assert.equal(both.resolved, true);
});

test('the walk never passes through a release branch', async () => {
  const first = await resolve('v10.1', { 'v10.1': [[1, 'v10.2']] });
  assert.equal(first.base, 'v10.1');
  assert.equal(first.calls.length, 0);

  const later = await resolve('fix/a', { 'fix/a': [[1, 'v10.1']], 'v10.1': [[2, 'v10.2']] });
  assert.equal(later.base, 'v10.1');
  assert.equal(later.calls.length, 1);
});

test('a branch that heads no open PR is where the stack lands', async () => {
  const r = await resolve('fix/b', { 'fix/b': [[2, 'feature/long-lived']] });
  assert.equal(r.base, 'feature/long-lived');
  assert.equal(r.resolved, true);
  assert.equal(r.chain, '#9 -> fix/b -> #2 -> feature/long-lived');
});

test('an API error propagates instead of resolving', async () => {
  const github = {
    paginate: async () => Promise.reject(new Error('rate limited')),
    rest: { pulls: { list: async () => ({ data: [] }) } },
  };
  await assert.rejects(
    resolveStackBase({ github, owner: 'o', repo: 'r', pullRequest: { number: 9, base: { ref: 'fix/a' } } }),
    /rate limited/
  );
});

test('a base with several open PRs, or a cycle, is unresolved', async () => {
  const several = await resolve('fix/a', { 'fix/a': [[1, 'v10.2'], [2, 'main']] });
  assert.equal(several.resolved, false);
  assert.equal(several.chain, '#9 -> fix/a -> (head of several open PRs)');

  const cycle = await resolve('fix/a', { 'fix/a': [[1, 'fix/b']], 'fix/b': [[2, 'fix/a']] });
  assert.equal(cycle.resolved, false);
  assert.equal(cycle.chain, '#9 -> fix/a -> #1 -> fix/b -> #2 -> fix/a');
  assert.equal(cycle.calls.length, 1);
});
