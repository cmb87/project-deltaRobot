
 difference(){
hull(){
translate([0,0,0])translate([0,-16/2,0])cube([14,16,16]);      
translate([0,8+7+5,16/2+4])rotate([0,90,0])cylinder(r=13/2,h=14,$fn=30); 

}

translate([-1,0,(16-10.1)/2])translate([0,-10.1/2,0])cube([50,10.1,10.1]);  


translate([-1,8+7+5,16/2+4])rotate([0,90,0])cylinder(r=8/2,h=17,$fn=30); 


}
