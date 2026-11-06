'use strict';

const assert = require('node:assert/strict');
const test = require('node:test');
const { resolveManualImageTag } = require('./manual_image_tag.cjs');

test('main and release branches default to their own core image branch', () => {
  for (const branch of ['main', 'v10.2', 'v10.2.0']) {
    assert.equal(resolveManualImageTag({ ref: `refs/heads/${branch}` }), branch);
  }
});

test('feature branches require an explicit core image branch', () => {
  for (const imageTag of ['', '   ']) {
    assert.throws(
      () => resolveManualImageTag({ ref: 'refs/heads/fm/dual-arm-mobile-duo', imageTag }),
      /requires image_tag.*published MoveIt Pro.*main or v10\.2/
    );
  }
});

test('explicit image branches override the workspace branch', () => {
  for (const ref of ['refs/heads/main', 'refs/heads/fm/dual-arm-mobile-duo']) {
    for (const imageTag of ['main', 'v10.2', 'fm/core-feature']) {
      assert.equal(resolveManualImageTag({ ref, imageTag: ` ${imageTag} ` }), imageTag);
    }
  }
});

test('tag and missing refs require an explicit image branch', () => {
  for (const ref of ['', 'refs/tags/v10.2.0', 'refs/heads/main-feature']) {
    assert.throws(() => resolveManualImageTag({ ref }), /requires image_tag/);
    assert.equal(resolveManualImageTag({ ref, imageTag: 'main' }), 'main');
  }
});
