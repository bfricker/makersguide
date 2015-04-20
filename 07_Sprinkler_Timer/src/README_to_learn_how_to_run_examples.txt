Hi there!

In the src directory, you can place many files, but only one file can contain 
the main() function.

In order to create several examples per lesson, I have created multiple main_ 
source files that you must enable/disableso that you only have one file active 
at a time.

To enable or disable a main_ file, right-click on the file over in Project
Explorer.  Look for Resouce Configurations -> Exclude from Build ...
entry in the right-click menu.

In the box that appears, click on the enbale checkboxes next to the Debug
and Release builds.  You can only have one main_ file active at a time in 
order to build the project.

Enjoy!

David.

