LibPaths = []

env = Environment(CPPPATH=['-I/usr/include/python2.7'],
                  CXXFLAGS=['-O2', '-mfpmath=sse', '-msse4', '-march=native',
                            '-Wall', '-g', '-std=c++0x'],
                  LIBS=['boost_python', 'c'],
                  LIBPATH=LibPaths,
                  LINKFLAGS=['-pthread'])

bwpathfinder = env.SharedLibrary('bwpathfinder',
                                 Glob("*.cpp"))

Default(bwpathfinder)
