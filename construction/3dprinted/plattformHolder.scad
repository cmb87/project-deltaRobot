
//translate([50,0,+20])rotate([0,180,0])color("red")import("motor_nema17.stl");

difference(){
hull(){
translate([0,0,0])cylinder(r=43/2, h=12, $fn=40);
translate([30,-58/2,0])cube([38,5,12]);
}
translate([-20.0/2,-19.8/2,-1])cube([20.0,19.8,72]);  

translate([50,+70/2,6])rotate([90,0,0])cylinder(r=2, h=70, $fn=40);
translate([60,+70/2,6])rotate([90,0,0])cylinder(r=2, h=70, $fn=40);
translate([40,+70/2,6])rotate([90,0,0])cylinder(r=2, h=70, $fn=40);

translate([50,+70/2+15,6])rotate([90,0,0])cylinder(r=4, h=70, $fn=40);
translate([60,+70/2+15,6])rotate([90,0,0])cylinder(r=4, h=70, $fn=40);
translate([40,+70/2+15,6])rotate([90,0,0])cylinder(r=4, h=70, $fn=40);

translate([15,+70/2,6])rotate([90,0,0])cylinder(r=1.8, h=70, $fn=40);
translate([-15,+70/2,6])rotate([90,0,0])cylinder(r=1.8, h=70, $fn=40);

translate([-15,-5,6])rotate([90,0,0])cylinder(r=4, h=70, $fn=40);
translate([-15,70+5,6])rotate([90,0,0])cylinder(r=4, h=70, $fn=40);

translate([15,-5,6])rotate([90,0,0])cylinder(r=4, h=70, $fn=40);
translate([15,70+5,6])rotate([90,0,0])cylinder(r=4, h=70, $fn=40);


//translate([-30,0,-2])cube([120,100,42]); // Comment for low
translate([-30,-100,-2])cube([120,100,42]); // Comment for up

}



