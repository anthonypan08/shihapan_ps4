# Overall
Question 1 and 2 part 1 and 2 are all in the data loader file. Refer to the main function to run the correct question and part.
You can run all of them at the same time but I would recommend running one part at a time and comment out the other 3 parts. Obviously,
you would have to remake the file if you comment/uncomment new parts. Refer below regarding making a file.


# To Make The File
```console
$ cd _builds
$ make data_loader
```
Ensure that gtsam is properly installed and eigen3 is in
<b>
/usr/local/include 
</b>
. You don't need to install eigen3, just clone the project from github and place the folder there

# To Run The File
```console
$ ./data_loader
```
