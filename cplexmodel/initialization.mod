// Copyright (c) 2020 Tobias Kessler, Klemens Esterle
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// Intitialization
float RR[CarRange][CarRange];
//float Sqrt2RR[CarRange][CarRange];
float costheta[CarRange];
float sintheta[CarRange];
int NumCar2CarCollisions;
execute INITIALIZE 
{
	//Calculate radius - radius distance for each combination of vehicles	
	for (var r1 in CarRange)
	{
		for(var r2 in CarRange)
		{
 			RR[r1][r2] = CollisionRadius[r1] + CollisionRadius[r2];			
 			//Sqrt2RR[r1][r2] = Math.sqrt(2)*RR[r1][r2];
		}
	}	

	// calculate initial heading
	for (var c in CarRange)
	{
		sintheta[c] = Math.sin(Math.atan2(IntitialState[c][5],IntitialState[c][2]));
		costheta[c] = Math.cos(Math.atan2(IntitialState[c][5],IntitialState[c][2]));
	}	
	
	// agent2agent collision matrix is squared with dimension numcars-1 x numcars-1
	NumCar2CarCollisions = NumCars - 1;	
}
range car2carCollisionRange = 1..NumCar2CarCollisions;

