
all: link

link: build
	./ghcs.sh -shared -dynamic build/**/*.o -o lib/libHSroger.so

build: mkdirs deps
	./ghcs.sh -shared -dynamic -no-hs-main -fPIC -isrc -outputdir build --make src/Roger/HaskellExports.hs
	ghc -shared -dynamic -no-hs-main -fPIC -isrc -outputdir build cbits/HSwrapper.c
	rm a.out
	bash -O globstar -c "cp build/**/*.h include/"
	bash -O globstar -c "cp .cabal-sandbox/lib/**/*.so lib/"

mkdirs:
	mkdir -p build
	mkdir -p include
	mkdir -p lib

deps:
	cabal sandbox init
	cabal install Vec random mtl --enable-executable-dynamic

clean:
	rm -rf ./build
	rm -rf ./include
	rm -rf ./lib
	rm -rf ./.cabal-sandbox
	rm -f ./cabal.sandbox.config

