difference(){
translate([0,-11/2,-11/2])cube([50,11,11]);
translate([-1,0,0])rotate([0,90,0])cylinder(r=4.5,h=24,$fn=40);
translate([27,0,0])rotate([0,90,0])cylinder(r=8/2,h=27,$fn=40);
}