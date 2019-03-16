# CAD Files

This directory holds the CAD files for the waterproof enclosure that fits the Aquamote along with its battery and external pressure sensor. All the CAD design was done in Autodesk Inventor 2018, however the files should transfer to any CAD editor.

## File Description

There are three types of files in this directory.

<li>The first is the __\*.ipt file__. This is a proprietary filetype from Autodesk, and holds the rich information that defines the exact dimensions of the 3D object.</li>
<li>The second is the **\*.stl file**. This is an industry standard format for defining a 3D objects out of many triangles. The file is text-based and can be opened with just about any 3D editor. However, in transforming the real object into one created out of only triangles, there is some loss in fidelity. For example, a simple cylinder cannot be represented with triangles with 100% accuracy due to its curves.</li>
<li>The third type is the **\*.form** file. This file is used by the PreForm software that is used in conjuction with a 3D printer from FormLabs. In our case we used a Form 2, however other models are available. If you are not using the PreForm software to 3D print your files, or are machining them some alternate way, then you can safely ignore all the **\*.form** files.</li>


The __tests__ directory is not maintained, so you should only use it for reference.
