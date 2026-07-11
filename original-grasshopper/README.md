# Original Grasshopper implementation

This folder contains the original version of the simulation, built in Grasshopper (the visual programming environment for Rhino 3D).

| File | What it is |
| --- | --- |
| `simulating-the-street.gh` | The Grasshopper definition. This is the complete, runnable original. |
| `SimulatingTheStreet.cs` | The C# code from the scripting component at the heart of the definition, kept here as a plain file so the agent logic can be read without opening Rhino. |

## Important: the C# file is not standalone

`SimulatingTheStreet.cs` is the core agent logic (the pedestrian and vehicle classes, their state machines, and the simulation loop) written for a Grasshopper C# scripting component. It receives its inputs (spawn points, shop and station locations, street geometry, signal positions, scan distances) from other components wired up on the Grasshopper canvas, and it returns its outputs to the canvas for display. It cannot be compiled or run on its own.

## Running the original

If you have Rhino with Grasshopper, open `simulating-the-street.gh` and the whole definition, including this C# component, will run there.

If you do not have Rhino, use the browser port instead: the same behaviour was reimplemented in p5.js and runs live at [taishobuya.github.io/Simulating-the-Street](https://taishobuya.github.io/Simulating-the-Street/).
