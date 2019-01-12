#!/usr/bin/env bash
set -ex

brew update > /dev/null
brew bundle

pip install -U pytest
