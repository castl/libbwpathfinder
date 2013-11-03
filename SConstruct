#!/usr/bin/env python
# -*- coding: utf-8 -*-

LibPaths = ["."]

env = Environment(CPPPATH=['-I/usr/include/python2.7', "."],
                  CXXFLAGS=['-O2', '-mfpmath=sse', '-msse4', '-march=native',
                            '-Wall', '-g', '-std=c++0x'],
                  LIBS=['boost_python', 'c'],
                  LIBPATH=LibPaths,
                  LINKFLAGS=['-pthread'] + map(lambda x: "-Wl,-rpath=%s" % x, LibPaths))

bwpathfinder = env.SharedLibrary('bwpathfinder',
                                 Glob("*.cpp"))

Default(bwpathfinder)

test_env = env.Clone()
test_env.Append(LIBS=[bwpathfinder, "python2.7"])
for t in Glob("tests/*.cpp"):
    tp = test_env.Program(t)
    Default(tp)
