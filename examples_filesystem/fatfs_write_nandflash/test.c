int gkey;
 int WaitKey(int ms, int echo) 
{
	int key;
	if (echo) {
		key = ms & 0xFF;
	}
	return key;
}


