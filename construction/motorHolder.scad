
//translate([50,0,+20])rotate([0,180,0])color("red")import("motor_nema17.stl");

difference(){
hull(){
translate([0,0,0])cylinder(r=43/2, h=40, $fn=40);
translate([30,-58/2,0])cube([58,58,6]);
}
translate([-20.0/2,-20.0/2,-1])cube([20.0,20.0,72]);  

translate([25,-42.5/2,4])cube([67,42.5,50]);

translate([50,0,-5])cylinder(r=22.5/2, h=15, $fn=40);
translate([60+4,0,-5])cylinder(r=22.5/2, h=15, $fn=40);
translate([55-10/2,-22.5/2,-5])cube([14,22.5,20]);


translate([50+31/2,+31/2,-5])cylinder(r=3.5/2, h=15, $fn=40);
translate([50-31/2,+31/2,-5])cylinder(r=3.5/2, h=15, $fn=40);

translate([50-31/2,-31/2,-5])cylinder(r=3.5/2, h=15, $fn=40);
translate([50+31/2,-31/2,-5])cylinder(r=3.5/2, h=15, $fn=40);

translate([60+31/2+4,+31/2,-5])cylinder(r=3.5/2, h=15, $fn=40);
translate([60-31/2+4,+31/2,-5])cylinder(r=3.5/2, h=15, $fn=40);
translate([60-31/2+4,-31/2,-5])cylinder(r=3.5/2, h=15, $fn=40);
translate([60+31/2+4,-31/2,-5])cylinder(r=3.5/2, h=15, $fn=40);

translate([55-10/2+31/2,-3.5/2+31/2,-5])cube([14,3.5,60]);
translate([55-10/2+31/2,-3.5/2-31/2,-5])cube([14,3.5,60]);
translate([55-10/2-31/2,-3.5/2-31/2,-5])cube([14,3.5,60]);
translate([55-10/2-31/2,-3.5/2+31/2,-5])cube([14,3.5,60]);

}


