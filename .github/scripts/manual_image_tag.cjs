'use strict';

const { RELEASE_OR_MAIN } = require('./stack_base.cjs');

// Workspace feature branches do not imply a published MoveIt Pro image branch.
function resolveManualImageTag({ imageTag = '', ref = '' }) {
  const explicitTag = imageTag.trim();
  if (explicitTag) return explicitTag;

  const branch = ref.replace(/^refs\/heads\//, '');
  if (ref.startsWith('refs/heads/') && RELEASE_OR_MAIN.test(branch)) return branch;

  throw new Error(
    `Manual CI on '${ref}' requires image_tag: set it to the published MoveIt Pro ` +
    'image branch (for example, main or v10.2). The workspace branch does not select a core image.'
  );
}

module.exports = { resolveManualImageTag };
