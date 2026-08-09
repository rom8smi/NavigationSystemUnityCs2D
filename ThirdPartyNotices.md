This file will highlight what is being used from 3rd parties as well as links to relevant projects:

1. KDTree class (http://forum.unity3d.com/threads/29923-Point-nearest-neighbour-search-class) is derived, adapted and used for fast searches of nearest neighbours.

2. Delaunator Sharp library (MIT License, https://github.com/nol1fe/delaunator-sharp) was used as a base to derive and adapt delaunator triangulation code in this project. This includes optimizations, such as removing LINQ and refactoring the code so it would be compatible and easy to derive further for C++ implementation where needed.

3. Constrainautor was derived from javascript version of Constrainautor library (ISC License, https://github.com/kninnug/Constrainautor?tab=ISC-1-ov-file) and adapted to the project in order to constrain triangulation for navigation mesh obstacles.

4. Some methods in GenericCode namespace were adapted from StackOverflow questions. There are URL links above each of these methods which were used from StackOverflow questions.
