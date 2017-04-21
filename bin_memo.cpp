

int bin_memo(int n, int k, b){
	
	if (k == 1) ||( k == n-1) {
		return n;
	}

	else
	return binomial[n];
}
int binomial( int n int k){
	int bin[n][k]
	if (k ==0) || (k == n){
		return 1;
	}

	
	bin[n][k]= binomial(n-1, k-1) + binomial(n-1, k);
	return bin[n][k];

}