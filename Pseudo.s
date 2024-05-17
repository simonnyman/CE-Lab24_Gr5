if (red readings larger than both green and blue
and red - green > 100 and red - blue > 1000)
    Victim detected
else
    keep searching




if obstacle in center angels
    if obstacle is close
        smooth turn
    elif obstacle is too close  
        u-turn
elif obstacle in middle angles
    if obstacle is close
        smooth turn
    elif obstacle is too close  
        hard turn
elif obstacle in wide angles
    if obstacle is close
        smooth turn
    elif obstacle is too close  
        medium turn
else
    full speed forward
