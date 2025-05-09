void updateBotAngle()
{
    double positiveUpdate = 360.0 / //Degrees returned from manual right turn 
    double negativeUpdate = 360.0 / //Degrees returned from manual left turn as a positive number

    static double lastAngle = 0.0;

    while(1)
    {
        double currentAngle = SENSOR.heading();
        double changeInAngle = currentAngle - lastAngle;

        if(changeInAngle < 0)
        {
            glblBotAngle += negativeUpdate * changeInAngle;
        }
        else
        {
            glblBotAngle += positiveUpdate * changeInAngle;
        }

        lastAngle = currentAngle;
        task::sleep(5);
    }
}