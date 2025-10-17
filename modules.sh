#!/bin/bash
# Update all submodules to their remote 'main' branch

git submodule foreach '
  echo "Updating $name..."
  git fetch origin main
  git checkout main
  git pull origin main
'
