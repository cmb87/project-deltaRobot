
difference(){
hull(){
translate([-42/2+3,-42/2+3,0])cylinder(r=3,h=4.5*2+3.5,$fn=30);
translate([+42/2-3,-42/2+3,0])cylinder(r=3,h=4.5*2+3.5,$fn=30);
translate([-42/2+3,+42/2-3,0])cylinder(r=3,h=4.5*2+3.5,$fn=30);
translate([+42/2-3,+42/2-3,0])cylinder(r=3,h=4.5*2+3.5,$fn=30);
};
translate([0,0,-1])cylinder(r=28/2,h=15,$fn=30);

translate([-31/2,-31/2,-1])cylinder(r=2.5,h=15,$fn=30);
translate([+31/2,-31/2,-1])cylinder(r=2.5,h=15,$fn=30);
translate([+31/2,+31/2,-1])cylinder(r=2.5,h=15,$fn=30);
translate([-31/2,+31/2,-1])cylinder(r=2.5,h=15,$fn=30);





translate([-35/2,0,-3])cylinder(r=2,h=17,$fn=30);
translate([+35/2,0,-3])cylinder(r=2,h=17,$fn=30);
translate([0,-35/2,-3])cylinder(r=2,h=17,$fn=30);
translate([0,+35/2,-3])cylinder(r=2,h=17,$fn=30);

}

