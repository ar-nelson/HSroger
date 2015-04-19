#!/bin/bash
# Taken from https://gist.github.com/cstrahan/8984109

DIR=$PWD
TARGET="cabal.sandbox.config"
while [ ! -e $DIR/$TARGET -a $DIR != "/" ]; do
  DIR=$(dirname $DIR)
done
if test $DIR != "/"; then
  DB=$(cabal sandbox hc-pkg list 2> /dev/null | grep \: | tac | sed 's/://' | paste -d: - -)
  GHC_PACKAGE_PATH=$DB ghc "$@"
else
  ghc "$@"
fi

