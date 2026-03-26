#!/usr/bin/env bash
set -euo pipefail

# Usage: ./release.sh 0.2.0

VERSION="${1:?Usage: ./release.sh <version>}"
TAG="v$VERSION"

# Check we're on main
BRANCH=$(git rev-parse --abbrev-ref HEAD)
if [ "$BRANCH" != "main" ]; then
  echo "Error: must be on main branch (currently on $BRANCH)"
  exit 1
fi

# Check working tree is clean
if ! git diff-index --quiet HEAD --; then
  echo "Error: working tree is dirty, commit or stash changes first"
  exit 1
fi

# Check tag doesn't already exist
if git rev-parse "$TAG" >/dev/null 2>&1; then
  echo "Error: tag $TAG already exists"
  exit 1
fi

# Update Cargo.toml version
sed -i "s/^version = \".*\"/version = \"$VERSION\"/" Cargo.toml

# Verify it parses
cargo check --quiet

# Commit and tag
git add Cargo.toml Cargo.lock
git commit -m "release: $TAG"
git tag "$TAG"

echo ""
echo "Created commit and tag $TAG"
echo "Run 'git push && git push --tags' to trigger the release"