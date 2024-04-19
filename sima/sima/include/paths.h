int path;
bool colorBlue;

void drivePath()
{
	startMillis = millis();

	switch (path)
	{
		case 1:
			drive(1725);
			turn90(colorBlue?'r':'l');
			drive(5000);
			break;
		
		case 2:
			delay(1000);
			drive(1500);
			turn90(colorBlue?'r':'l');
			drive(2850);
			break;

		case 3:
			delay(2000);
			drive(1000);
			turn90(colorBlue?'r':'l');
			drive(2500);
			break;
		
		case 4:	
      pwmOffset = colorBlue?pwmOffset+1:pwmOffset-3;
			driveFast(8000);
			break;

		// Test cases
		case 11:
			drive(5000);
			break;
		case 12:
			turn90('r');
			break;
		case 13:
			turn90('l');
			break;
		default:
			break;
	}
}