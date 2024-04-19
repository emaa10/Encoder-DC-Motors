void drivePath(int path)
{
    startMillis = millis();

    switch (path)
    {
        case 1:
            drive(1000, 9, 180, startMillis);
            turn90('r', startMillis);
            drive(5000, 9, 180, startMillis);
            stop();
            break;
        
        case 2:
            drive(1000, 90, 180, startMillis);
            break;

        default:
            break;
    }
}