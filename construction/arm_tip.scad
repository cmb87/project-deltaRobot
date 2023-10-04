difference(){
hull(){
translate([0,-27/2,-27/2])cube([20,27,27]);
translate([40,20/2,0])rotate([90,00,0])cylinder(r=9, h=20, $fn=20);
}
translate([-1,-20/2,-20/2])cube([25,20,20]);
translate([40,40/2,0])rotate([90,00,0])cylinder(r=8, h=40, $fn=20);
}

difference(){
translate([40,22/2,0])rotate([90,00,0])cylinder(r=8, h=22, $fn=20);
translate([40,60/2,0])rotate([90,00,0])cylinder(r=4, h=60, $fn=20); 
}