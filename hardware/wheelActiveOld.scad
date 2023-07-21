
z = 8;
t = 7.8;
d = (z-1)*t/3.14159265359;
w = 3.7;
b = 4;

echo(d);


difference(){
  translate([0,0,-12/2])cylinder(r=d/2, h=12, $fn=40);
  translate([0,0,-14/2])cylinder(r=3.9/2, h=14, $fn=42);

    translate([0,0,12/2+0.01]){
      hull(){
        translate([0,0,0])cylinder(r=(d-3)/2, h=0.1, $fn=42);
        translate([0,0,0-3])cylinder(r=(d-5)/2, h=1, $fn=42);
      }
    }



  for ( i = [0 : 1 : z] ){
    
    delta = (360/z);
    
    rotate( i * delta, [0, 0, 1])
    translate([-(b+0.7)/2,d/2-3.5,-w/2])cube([b+0.7,d+10,w]);
}

}

translate([-3/2,-3/2+2.8,-12/2])cube([3,3,9]);