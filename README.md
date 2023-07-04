
# KiCad
 - to make the file load pcbnew and then do File->Fabrication Outputs.  save a .pos file {CSV, millimeters, seperate files for front/back} -> Generate Position File.  (chgray-keyboard-top-pos.csv)
	A) Change Ref to Designator
	B) Rot to Rotation
	C) Side to Layer
	D) PoxX to "Mid X"
	E) PosY to "Mid Y"
 
 - to make the BOM load eeschema and then do the button ("generate bill of material") BOM -> bom2csv
     you're looking for chgray-keyboard.csv
	A) Change Reference to "Designator"
	B) Change Datasheet to "JLCPCB Part #"
	C) Value to Comment