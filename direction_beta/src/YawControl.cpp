


void (){

double betha, tetha, K;
K=180;
	if (targetA >= 0){ // Cuando target es positivo
		if (currentA>=0)
			vel=(targetA-currentA)/K; // 
		else{
			tetha = abs(currentA)+targetA;
			betha = 360 - tetha;
			if(betha>tetha)
				vel = tetha/K; // vel+
			else
				vel = -betha/K; // vel-
		}
	}
	else{ // Cuando target es negativo
		if (currentA<0)
			vel = -(abs(targetA)-abs(currentA))/K; //
		else{
			tetha = currentA+abs(targetA);
			betha = 360 - tetha;
			if(betha>tetha)
			vel = -tetha/K; // vel -
			else
			vel = betha/K; // vel +
		}
	}
}
