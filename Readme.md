# FPC cyclone-physics

>
> As i got stuck in writing my own 3D physics engine [CorP3D](https://github.com/PascalCorpsman/CorP3D) i decided to "recode" a existing one in order to learn enough to finish CorP3D.
>

FPC cyclone-physics is a 3D physics game engine. It is developed in the book [Game Physics Engine Development, second edition](https://www.amazon.de/-/en/Game-Physics-Engine-Development-Commercial-Grade/dp/0123819768).

Ian Millington starts from scratch in his book, and so do i here in this repository. The plan is to read trough the book and implement step by step the whole engine in FreePascal (aka crossporting the C++ code given in the book).

Goal is to have at the end a fully working 3D physics library (the same as in the book) and also port the [Demos](src/Demos).

There are eleven demo's available in total. The first seven demos are part of the particle engine and the last four for the rigid body physics.

| Number | Page in Book | Name
| ---    | ---          | ---
| 001    |  63          | [ballistic](src/Demos/001_ballistic)
| 002    |  66          | [fireworks](src/Demos/002_fireworks)
| 003    | 151          | [bridge](src/Demos/003_bridge)
| 004    | 152          | [platform](src/Demos/004_platform)
| 005    | 153          | [blob](src/Demos/005_blob)
| 006    | 241          | flightsim
| 007    | 247          | sailboat
| 008    |  65          | bigballistic
| 009    | 441          | [ragdoll](src/Demos/009_ragdoll)
| 010    | 446          | fracture
| 011    | 457          | [explosion](src/Demos/011_explosion)

If you do not wan't to wait up until i finished reading the book you can try look at the books github repository which is located [here](https://github.com/idmillington/cyclone-physics).

### What is needed to compile and run the code ?

1. clone this repository

Now you have everything to compile the engine.

### What is needed to compile and run the demos ?

1. install LazOpenGLControl into the Lazarus IDE
2. download [dglOpenGL.pas](https://github.com/SaschaWillems/dglOpenGL/blob/master/dglOpenGL.pas)

### Progress
- 2025.05.23: created repository, readme.md and license.md
- 2025.05.25: demo 001 ballistic
- 2025.05.26: demo 002 fireworks
- 2025.10.26: demo 009 ragdoll
- 2025.10.30: demo 011 explosion
- 2025.10.31: demo 003 bridge
- 2025.11.01: demo 004 platform
- 2025.11.02: demo 005 blob
