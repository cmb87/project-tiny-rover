
d = 17.3797;


echo(d);


difference(){

  union(){
    translate([0,0,+1.5/2+1.5])cylinder(r=d/2, h=6-(1.5/2+1.5), $fn=40);
    hull(){
      translate([0,0, +1.5/2+1.5])cylinder(r=(d-0)/2, h=0.1, $fn=42);
      translate([0,0, +1.5/2])cylinder(r=(d-5)/2, h=0.1, $fn=42);
    }
    translate([0,0,-1.5/2])cylinder(r=(d-5)/2, h=1.5, $fn=42);

    rotate([0,180,0])hull(){
      translate([0,0,  +1.5/2+1.5])cylinder(r=(d-0)/2, h=0.1, $fn=42);
      translate([0,0,  +1.5/2])cylinder(r=(d-5)/2, h=0.1, $fn=42);
    }
    rotate([0,180,0])translate([0,0,+1.5/2+1.5])cylinder(r=d/2, h=6-(1.5/2+1.5), $fn=40);
  }
    
    translate([0,0,12/2+0.01]){
      hull(){
        translate([0,0,0])cylinder(r=(d-3)/2, h=0.1, $fn=42);
        translate([0,0,0-3])cylinder(r=(d-5)/2, h=1, $fn=42);
      }
    }
    translate([0,0,-14/2])cylinder(r=3.5, h=14, $fn=30);
    translate([0,0,-12/2-0.1])cylinder(r=10.15/2, h=17.1, $fn=30);
}





// 

z = 8;
for ( i = [0 : 1 : z] ){
rotate([0,0,i*360/z])translate([17.3797/2-3/2,0,-5/2])cylinder(r=3/2, h=5, $fn=42);
}
