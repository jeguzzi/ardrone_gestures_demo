
// PID

double errorant=0, ierrorant =0, ts = (1/Nodefrequency), Kp, Ki, Kd;


double PID (double targetY, double actualY){

double error, ierror , ts, outPID;

error =targetY-actualY;
ierror = ierrorant + error*ts
// AntiWindUp

// Option 1
	ierror = min("maximum value of ierror",ierror);
	ierror = max("minumum value of ierror",ierror);

// Option 2
	if (ierror > "maximum value of ierror" || ierror < "minimum value of ierror")
		ierror = 0.0;

outPID = Kp*error+Ki*ierror+Kd*((error-errorant)/ts)

errorant = error;
ierrorant = ierror;

return outPID;
}
