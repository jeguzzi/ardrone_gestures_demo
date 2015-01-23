void 
testQuaternion(double qx, double qy, double qz, double qw,
		    double &yaw, double &pitch, double &roll)
{ 
    double sqw = qw*qw; 
    double sqx = qx*qx; 
    double sqy = qy*qy; 
    double sqz = qz*qz; 

    double m[9];
    //invs (inverse square length) is only required if quaternion is not already normalised 
    double invs = 1 / (sqx + sqy + sqz + sqw); 
    
    m[0] = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs 
    m[4] = (-sqx + sqy - sqz + sqw)*invs ; 
    m[8] = (-sqx - sqy + sqz + sqw)*invs ; 
    
    double tmp1 = qx*qy; 
    double tmp2 = qz*qw; 
    m[3] = 2.0 * (tmp1 + tmp2)*invs ; 
    m[1] = 2.0 * (tmp1 - tmp2)*invs ; 
    
    tmp1 = qx*qz; 
    tmp2 = qy*qw; 
    m[6] = 2.0 * (tmp1 - tmp2)*invs ; 
    m[2] = 2.0 * (tmp1 + tmp2)*invs ; 
    tmp1 = qy*qz; 
    tmp2 = qx*qw; 
    m[7] = 2.0 * (tmp1 + tmp2)*invs ; 
    m[5] = 2.0 * (tmp1 - tmp2)*invs ; 

    yaw = atan2(-m[6],m[0]); 
    pitch = asin(m[3]); 
    roll = atan2(-m[5],m[4]); 
}


double DirectionNpostion(double Lx,double Ly, double Nx, dobule Ny){

double vec = [Nx-Lx, Ny-Ly];
if(vec[0]>=0 && vec[1]>0)
	vecYaw = atan(y,x);
else if (vec[0]<0 && vec[1]>=0)
	vecYaw = atan(y,x)+90;
else if (vec[0]=<0 && vec[1]<0)
	vecYaw = atan(y,x)+180;
else if (vec[0]>0 && vec[1]=<0)
	vecYaw = atan(y,x)+270;
else 
	vecYaw = 0.0;

return vecYaw;
}



void quaternion2angles(){
}
