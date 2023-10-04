homeAngle = 45 ;


if(true){
difference(){

hull(){
translate([0,0,0])cylinder(r=35/2, h=1, $fn=40);
translate([0,0,9])cylinder(r=35/2, h=1, $fn=40);
rotate([0,0,homeAngle])translate([-30,-20/2,0])cube([30,20,4]); 
}

translate([-20/2,-20/2,-1])cube([20,20,42]);   
 
}

rotate([0,0,homeAngle])
difference(){
    
hull(){
translate([-52+3,-20/2+3,0])cylinder(r=6/2, h=4, $fn=40);
translate([-52+3, 20/2-3,0])cylinder(r=6/2, h=4, $fn=40);
translate([-18+3,-20/2+3,0])cylinder(r=6/2, h=4, $fn=40);
translate([-18+3, 20/2-3,0])cylinder(r=6/2, h=4, $fn=40);
}

    rotate([0,0,-homeAngle])translate([-20/2,-20/2,-1])cube([20,20,42]);
    
    translate([-52+10-(15-3.2-3.2/2)/2,+5-3.2/2,-1])cube([15-3.2-3.2/2,3.2,15]); 
    translate([-52+10-(15-3.2-3.2/2)/2,-5-3.2/2,-1])cube([15-3.2-3.2/2,3.2,15]); 
    
    for ( i = [1 : 3] ){
        x = -52 + 5*i;
        translate([x,-5,-1])cylinder(r=3.2/2, h=15, $fn=40);
        translate([x,+5,-1])cylinder(r=3.2/2, h=15, $fn=40);
    }
}

}





//t