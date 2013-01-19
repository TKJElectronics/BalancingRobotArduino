void initDropdownlist() {
  COMports = controlP5.addDropdownList("COMPort", 10, 20, 210, 200); // Make a dropdown list with all comports
  customize(COMports); // Setup the dropdownlist by using a function

  controlP5.addButton("Connect", 0, 225, 3, 45, 15);
  controlP5.addButton("Disconnect", 0, 275, 3, 52, 15);
}

void customize(DropdownList ddl) {
  ddl.setBackgroundColor(color(200));//Set the background color of the line between values
  ddl.setItemHeight(20);//Set the height of each item when the list is opened.
  ddl.setBarHeight(15);//Set the height of the bar itself.

  ddl.captionLabel().style().marginTop = 3;//Set the top margin of the lable.  
  ddl.captionLabel().style().marginLeft = 3;//Set the left margin of the lable.  
  ddl.valueLabel().style().marginTop = 3;//Set the top margin of the value selected.
  
  if (ddl.name() == "COMPort") {
    ddl.captionLabel().set("Select COM port");//Set the lable of the bar when nothing is selected. 
    //Now well add the ports to the list, we use a for loop for that.
    for (int i=0; i<Serial.list().length; i++)    
      ddl.addItem(Serial.list()[i], i);//This is the line doing the actual adding of items, we use the current loop we are in to determin what place in the char array to access and what item number to add it as.
  }
  ddl.setColorBackground(color(60));
  ddl.setColorActive(color(255, 128));
}

