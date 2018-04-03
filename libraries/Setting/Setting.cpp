#include "Setting.h"

Setting::Setting(String settingname, double initV, double minV, double maxV, int knobstepV)
{
  name = settingname;
  value = initV;
  min = minV;
  max = maxV;
  knobstep = knobstepV;
  update(0);
}

void Setting::update(int knobdelta)
{
    double change = knobdelta/knobstep;
    value += change;
    value = (value<min) ? min:value;
    value = (value>max) ? max:value;
}

void Setting::setMax(double maxV)
{
    max = maxV;
    update(0);
}

void Setting::setMin(double minV)
{
    min = minV;
    update(0);
}
