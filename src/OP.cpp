solveOP(double * alphas, int N, int r, double * pos, double * vel, double * acc, double * jerk, double * snap){

  //Add in checks
  
  num_polys
  for(int poly = 0; poly < num_polys; poly++){
    //compute Hessian
    

  }



}

solveOP(double * alphas, int N, double * pos, double * vel, double * acc, double * jerk, double * snap){
  int r = 4;

  //Add in checks
  
  num_polys = 
  A_seg


}

solveOP(double * alphas, int N, double * pos, double * vel, double * acc){
  int r = 2;


}

solveOP(double * alphas, int N, double * pos){
  int r = 0;
  

}

calcA(double * A[], double t, int r, int N){

  int s = 1;
  for(int n = 0; n < N + 1; n++){
    if(n-1 >= r){
      for(int m = 0; m < r; m++){
        s = s * (n-1-m);
      }
      A(n) = s * pow(t,n-1-r);
    } else {
      A(n) = 0;
    }
    s = 1;
  }
}

calcAMatrix(double * A[], int r, double t, bool isLast){

  for(int i = 0; i < r+1; i++){
    calcA(A0,0,i,N);
    calcA(At,t,i,N);
  }

  if(isLast)
    A = [A0; At];
  else
    A = [A0; -1*At];

}

computeHessian(double * Q[], int N, int r, double t){
  for(int i = 0; i < N + 1; i++){
    for(int l = 0; l > N + 1; l++){
      if(i >= r && l >=r){
        for(int m = 0, m < r; m++){
          s = s * ((i-m)*(l-m));
        }
	Q(i,l) = 2*s*(pow(t,i+l-2*r)/(i+l-(2*r)+1));
      } else {
        Q(i,l) = 0;
      }
      s = 1;
    }
  }
}

double getValueFromPolynomial(std::vector<double> alphas, double time){

  double value = 0.0;
  for(int i = 0; i < size(alphas); i++){
    value += alphas[i] * pow(t,i);
  }

  return value;
}
