//// UAVPSet
// Copyright (C) 2007  Thorsten Raab
// Email: thorsten.raab@gmx.at
// Michael Sachs
// Email: michael.sachs@online.de
// 
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

using System;
using System.Collections.Generic;
using System.Text;
using System.Resources;

namespace UAVP.UAVPSet
{
    static class Hilfe
    {
        static public void info(FormMain mainForm)
        {
                // Roll     
                if (mainForm.RollProp1NumericUpDown.Focused || mainForm.RollProp2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Proportional");
                if (mainForm.RollDiff1NumericUpDown.Focused || mainForm.RollDiff2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Differential");
                if (mainForm.RollInt1NumericUpDown.Focused || mainForm.RollInt2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Integral");
                if (mainForm.RollIntLimit1NumericUpDown.Focused || mainForm.RollIntLimit2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("IntegralLimiter");
                if (mainForm.RollAcc1NumericUpDown.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Neutral");
                
                // Pitch
                if (mainForm.PitchProp1NumericUpDown.Focused || mainForm.PitchProp2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Proportional");
                if (mainForm.PitchDiff1NumericUpDown.Focused || mainForm.PitchDiff2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Differential");
                if (mainForm.PitchInt1NumericUpDown.Focused || mainForm.PitchInt2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Integral");
                if (mainForm.PitchIntLimit1NumericUpDown.Focused || mainForm.PitchIntLimit2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("IntegralLimiter");
                if (mainForm.PitchAcc1NumericUpDown.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Neutral");

                if (mainForm.Balance1NumericUpDown.Focused || mainForm.Balance2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Balance");
                            
                // Yaw
                if (mainForm.YawProp1NumericUpDown.Focused || mainForm.YawProp2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Proportional");
              
                if (mainForm.YawInt1NumericUpDown.Focused || mainForm.YawInt2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Integral");
                if (mainForm.YawLimit1NumericUpDown.Focused || mainForm.YawLimit2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Limiter");
                if (mainForm.YawIntLimit1NumericUpDown.Focused || mainForm.YawIntLimit2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("IntegralLimiter");
              
                if (mainForm.VertAcc1NumericUpDown.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Neutral");

            // General
                if (mainForm.bit01CheckBox.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("AuxMode");
                if (mainForm.bit61CheckBox.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("AccOrientation");
                if (mainForm.bit21CheckBox.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("TxMode");
                if (mainForm.bit31CheckBox.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("RxFormat");
                if (mainForm.bit41CheckBox.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("RangefinderResolution");
                if (mainForm.bit51CheckBox.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("GPSAltitude");
                if (mainForm.bit11CheckBox.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("RTHDescend");

                if (mainForm.HoverThrottle1NumericUpDown.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("HoverThrottle");

                if (mainForm.BaroScale1NumericUpDown.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("BaroScale");
                if (mainForm.LowMotorRun1NumericUpDown.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("LowMotorRun");
                if (mainForm.CameraRoll1NumericUpDown.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("CameraGain");
                if (mainForm.CameraRollTrim1NumericUpDown.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("CameraRollTrim");
                if (mainForm.Compass1NumericUpDown.Focused || mainForm.Compass2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Compass");
                if (mainForm.CompassOffset1NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("CompassOffset");
                if (mainForm.Battery1NumericUpDown.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Unterspannung");
                if (mainForm.AltProp1NumericUpDown.Focused || mainForm.AltProp2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("AltProp");
                if (mainForm.AltInt1NumericUpDown.Focused || mainForm.AltInt2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("AltInt");
                if (mainForm.AltIntLimit1NumericUpDown.Focused || mainForm.AltIntLimit2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("AltIntLimit");
                
                if (mainForm.Orientation1NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("XModus");

                if (mainForm.BatteryCapacity1NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("BatteryCapacity");

                if (mainForm.AltIntLimit1NumericUpDown.Focused || mainForm.AltIntLimit2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("AltIntLimit");

                if (mainForm.ThrottleGS1NumericUpDown.Focused || mainForm.ThrottleGS2NumericUpDown.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("ThrottleGainSchedule");

                if (mainForm.Acro1NumericUpDown.Focused || mainForm.Acro2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("Acro");

                if (mainForm.DescDelay1NumericUpDown.Focused || mainForm.DescDelay2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("DescentDelay");
                if (mainForm.DescentRate1NumericUpDown.Focused || mainForm.DescentRate2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("DescentRate");
                if (mainForm.BaroAccComp1NumericUpDown.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("BaroFilter");

                if (mainForm.GPSGain1NumericUpDown.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("GPSGain");

                if (mainForm.GyroComboBox1.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("GyroType");
                if (mainForm.ESCComboBox1.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("ESCType");
                if (mainForm.TxRxComboBox1.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("TxRxType");

                if (mainForm.TelemetryComboBox1.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("TelemetryType");
  
                //GPS

                if (mainForm.NavNeutralRadius1NumericUpDown.Focused || mainForm.NavNeutralRadius2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("NavNeutralRadius");
                if (mainForm.NavInt1NumericUpDown.Focused || mainForm.NavInt2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("NavInt");
                if (mainForm.NavDiff1NumericUpDown.Focused || mainForm.NavDiff2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("NavDiff");
                if (mainForm.NavIntLimit1NumericUpDown.Focused || mainForm.NavIntLimit2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("NavIntLimit");
                if (mainForm.NavRTHAlt1NumericUpDown.Focused || mainForm.NavRTHAlt2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("NavRTHAltitude");
                if (mainForm.NavMagVar1NumericUpDown.Focused )
                    mainForm.infoTextBox.Text = mainForm.help.GetString("NavMagVar");

                if (mainForm.NavGPSSlew1NumericUpDown.Focused || mainForm.NavGPSSlew2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("NavGPSSlew");

                if (mainForm.NavYawLimit1NumericUpDown.Focused || mainForm.NavYawLimit2NumericUpDown.Focused)
                    mainForm.infoTextBox.Text = mainForm.help.GetString("NavTurnLimit");
        }
    }
}
