// =================================================================================================
// =                                  UAVX Quadrocopter Controller                                 =
// =                             Copyright (c) 2008 by Prof. Greg Egan                             =
// =                         Instruments Copyright (c) 2008 Guillaume Choutea                      =                                        */
// =                               http://code.google.com/p/uavp-mods/                             =
// =================================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU 
//    General Public License as published by the Free Software Foundation, either version 3 of the 
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without even 
//    the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
//    General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;

namespace UAVXGS
{
    public partial class FormMain : Form
    {
        // ASCII Constants
        const byte NUL = 0;
        const byte SOH = 1;
        const byte EOT = 4;
        const byte ACK = 6;
        const byte HT = 9;
        const byte LF = 10;
        const byte CR = 13;
        const byte NAK = 21;
        const byte ESC = 27;

        const Byte RCMaximum = 238;

        const byte UnknownPacketTag = 0;
        const byte RestartPacketTag = 8;
        const byte UAVXFlightPacketTag = 13;
        const byte UAVXNavPacketTag = 14;

        const byte WaitRxSentinel = 0;
        const byte WaitRxBody = 1;
        const byte WaitRxESC = 2;
        const byte WaitRxCheck = 3;
        const byte WaitRxCopy = 4;
        const byte WaitUPTag = 5;
        const byte WaitUPLength = 6;
        const byte WaitUPBody = 7;

        const byte DefaultAttitudeToDegrees = 35;
        byte AttitudeToDegrees;

        const int DefaultRangeLimit = 100;
        const int MaximumRangeLimit = 250; // You carry total responsibility if you increase this value
        const int MaximumAltitudeLimit = 121; // You carry total responsibility if you increase this value 

        const double BatteryChargeScale = 1; // depends on current ADC used - possible needs calibration box?

        /*
        UAVXFlightPacket
        
        Flags bit values
        Flags[0]
            NavAltitudeHold	
            TurnToWP			
            MotorsArmed
            LostModel
            NearLevel
            LowBatt
            GPSValid
            NavValid

        Flags[1]
            BaroFailure
            AccFailure
            CompassFailure
            GPSFailure
            AttitudeHold
            ThrottleMoving
            Hovering
            Navigate
                
        Flags[2]
            ReturnHome
            Proximity
            CloseProximity
            UsingGPSAlt
            UsingRTHAutoDescend
            BaroAltitudeValid
            RangefinderAltitudeValid
            UsingRangefinderAlt
      
        // internal flags not really useful externally
        Flags[3..5]
            Signal
            RCFrameOK
            ParametersValid
            RCNewValues
            NewCommands
            AccelerationsValid
            CompassValid
            CompassMissRead

            GyrosErected
            ReceivingGPS
            GPSSentenceReceived
            NavComputed
            CheckThrottleMoved		
            WayPointsChanged
            UsingSerialPPM
            UsingTxMode2

            UsingXMode
            UsingTelemetry
            TxToBuffer
            NewBaroValue
            BeeperInUse
            AcquireNewPosition 
            GPSTestActive
            unused
            */

        // byte UAVXFlightPacketTag;   
        // byte Length;  
        const byte NoOfFlagBytes = 6;
        byte[] Flags = new byte[NoOfFlagBytes];
        byte StateT;                    // 8
        short BatteryVoltsT;            // 9
        short BatteryCurrentT;          // 11
 		short BatteryChargeT;           // 13
        short RCGlitchesT;              // 15
        short DesiredThrottleT;         // 17
        short DesiredRollT;             // 19
        short DesiredPitchT;            // 21
        short DesiredYawT;              // 23
        short RollRateT;                // 25
        short PitchRateT;               // 27
        short YawRateT;                 // 29
        short RollSumT;                 // 31
        short PitchSumT;                // 33
        short YawSumT;                  // 35
        short LRAccT;                   // 37
        short FBAccT;                   // 39
        short DUAccT;                   // 41
        short LRCompT;                  // 43
        short FBCompT;                  // 44
        short DUCompT;                  // 45
        short AltCompT;                 // 46
   
        // UAVXNavPacket
        //byte UAVXNavPacketTag;
        //byte Length;
        byte NavStateT;                 // 2
        byte FailStateT;                // 3
        byte GPSNoOfSatsT;              // 4
        byte GPSFixT;                   // 5

        byte CurrWPT;                   // 6

        short BaroROCT;                 // 7 
        int RelBaroAltitudeT;           // 9

        short RangefinderROCT;          // 12
        short RangefinderAltitudeT;     // 14

        short GPSHDiluteT;              // 16
        short HeadingT;                 // 18
        short DesiredCourseT;           // 20

        short GPSVelT;                  // 22
        short GPSROCT;                  // 24
        int GPSRelAltitudeT;            // 26
        int GPSLatitudeT;               // 29
        int GPSLongitudeT;              // 33
        int DesiredAltitudeT;           // 37
        int DesiredLatitudeT;           // 40
        int DesiredLongitudeT;          // 44
        int NavStateTimeoutT;          // 48;

        double Distance, LongitudeCorrection, WhereDirection;
        bool FirstGPSCoordinates = true;
        int OriginLatitude, OriginLongitude;
        double EastDiff, NorthDiff;

        byte[] UAVXPacket = new byte[256];

        const short RxQueueLength = 2048;
        const short RxQueueMask = RxQueueLength - 1;
        byte[] RxQueue = new byte[RxQueueLength];
        short RxTail = 0;
        short RxHead = 0;

        short RxPacketByteCount;
        byte RxPacketTag, RxPacketLength, PacketLength, PacketRxState;
        byte ReceivedPacketTag;
        bool PacketReceived = false;

        bool CheckSumError;
        short RxLengthErrors = 0, RxCheckSumErrors = 0, RxIllegalErrors = 0;

        byte RxCheckSum;

        double CurrAlt, AltError;

        bool InFlight = false;

        public FormMain()
        {
            InitializeComponent();
        }

        public void SetSerialPort(string portSelected, int speed, ref string errorMessage)
        {
            try
            {
                errorMessage = "";
                serialPort1.PortName = portSelected;
                serialPort1.ReadBufferSize = 1024;
                serialPort1.BaudRate = speed;
                serialPort1.ReceivedBytesThreshold = 1;
                serialPort1.Open();
                serialPort1.Close();
            }
            catch (Exception er)
            {
                //MessageBox.Show(Convert.ToString(er.Message));
                errorMessage = Convert.ToString(er.Message);
            }
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (serialPort1.IsOpen) serialPort1.Close();
        }

        private void textBox1_KeyPress(object sender, KeyPressEventArgs e)
        {
            // If the port is closed, don't try to send a character.
            if (!serialPort1.IsOpen) return;

            // If the port is Open, declare a char[] array with one element.
            char[] buff = new char[1];

            // Load element 0 with the key character.
            buff[0] = e.KeyChar;

            // Send the one character buffer.
            serialPort1.Write(buff, 0, 1);

            // Set the KeyPress event as handled so the character won't
            // display locally. If you want it to display, omit the next line.
            e.Handled = true;
        }

       // private void cmdClearFlying_Click(object sender, EventArgs e)
        private void FlyingButton_Click(object sender, EventArgs e)
        {
            if (InFlight)
            {
                UAVXCloseTelemetry();
                FlyingButton.Text = "Disconnected";
                FlyingButton.BackColor = System.Drawing.Color.Red;
            }
            else
            {
                FlyingButton.Text = "Connected";
                FlyingButton.BackColor = System.Drawing.Color.Green;

                UAVXOpenTelemetry();
            }
            InFlight = !InFlight;
         //   e.Handled = true;
        }

        //-----------------------------------------------------------------------
        // UAVX Communications

        private void UAVXOpenTelemetry()
        {
            string sError = "";

            InitPollPacket();

            SetSerialPort("COM1", 9600, ref sError);
            if (sError != "")
            {
                //statusLabel.Text = sError;
                return;
            }
            else
            {
                // zero out Nav and Airframe packets?

                InitPollPacket();
 
            }
            serialPort1.Open();
            if (!serialPort1.IsOpen)
            {
                InFlight = false;
                return;
            }
        }

        private void UserAttitudeToDegrees_TextChanged(object sender, EventArgs e)
        {
            double outValue;

            if (double.TryParse(UserAttitudeToDegrees.Text, out outValue))
                AttitudeToDegrees = Convert.ToByte(outValue);
            else
                AttitudeToDegrees = DefaultAttitudeToDegrees;
        }


        private void serialPort1_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            while (serialPort1.BytesToRead != 0)
            {
                RxTail++;
                RxTail &= RxQueueMask;
                RxQueue[RxTail] = (byte)serialPort1.ReadByte();
            }
            this.Invoke(new EventHandler(UAVXReadTelemetry));
        }

        void InitPollPacket()
        {
              RxPacketByteCount = 0;   
              RxCheckSum = 0;

              Zero(ref UAVXPacket, 256);    
              RxPacketTag = UnknownPacketTag;

              RxPacketLength = 2; // set as minimum
              PacketRxState = WaitRxSentinel;    
        }

        void AddToBuffer(byte ch)
        {
            bool RxPacketError;

            UAVXPacket[RxPacketByteCount++] = ch;
            if (RxPacketByteCount == 1)
            {
                RxPacketTag = ch;
                switch ( RxPacketTag ) {
                case UAVXFlightPacketTag: 
                    PacketLength = 45; break;
                case UAVXNavPacketTag: PacketLength = 49; break;
                default:
                    RxIllegalErrors++;
                    RxPacketTag=UnknownPacketTag;  
                    break;
                } // switch
                if (RxPacketTag==UnknownPacketTag)
                    PacketRxState=WaitRxSentinel;
                else
                    PacketRxState=WaitRxBody;
            }
            else
                if (RxPacketByteCount == 2)
                {
                    RxPacketLength = ch; // ignore
                    PacketRxState = WaitRxBody;
                }
                else
                    if(RxPacketByteCount >= 255 ) // zzzsizeof(Packet)) // packet too long for buffer
                    {
                        RxLengthErrors++;
                        PacketRxState = WaitRxSentinel;
                    }
                    else
                        if (RxPacketByteCount >= (PacketLength+3))
                        {
                            RxPacketError = RxCheckSum != 0;

                            CheckSumError = RxPacketError;
                            if (CheckSumError)
                                RxCheckSumErrors++;

                            RxPacketError=false;
                            if (!RxPacketError)
                            {
                                PacketReceived = true;
                                ReceivedPacketTag=RxPacketTag;   
                            }
                            PacketRxState = WaitRxSentinel;
                         //   InitPollPacket(); 
                        }
                        else
                            PacketRxState = WaitRxBody;         
        }

        void ParsePacket(byte ch) 
        {
            switch (PacketRxState) {
            case WaitRxSentinel:
                if (ch == SOH)
                {
                    InitPollPacket();
                    CheckSumError = false;
                    PacketRxState = WaitRxBody;
                }
                break;
            case WaitRxBody:
               if (ch == ESC)
                   PacketRxState = WaitRxESC;
                else
                    if (ch == SOH) // unexpected start of packet
                    {
                        RxLengthErrors++;

                        InitPollPacket();
                        PacketRxState = WaitRxBody;
                    }
                    else
                        if (ch == EOT) // unexpected end of packet 
                        {
                            RxLengthErrors++;
                            PacketRxState = WaitRxSentinel;
                        }
                        else
                            AddToBuffer(ch); 
                break;
            case WaitRxESC:
                AddToBuffer(ch);
                break;
            default: PacketRxState = WaitRxSentinel; break; 
            } 
        }

        public void UAVXReadTelemetry(object sender, EventArgs e)
        {
            byte b;
            short i;
    
          //  textBox1.AppendText(b);

            while ( RxHead != RxTail) 
            {
                b = RxQueue[RxHead];
                RxHead++;
                RxHead &= RxQueueMask;

                ParsePacket( b );
                
                RxTypeErr.Text = string.Format("{0:n0}", RxIllegalErrors);
                RxCSumErr.Text = string.Format("{0:n0}", RxCheckSumErrors);
                RxLenErr.Text = string.Format("{0:n0}", RxLengthErrors);

                if ( PacketReceived )
                {
                    PacketReceived = false;

                    AttitudeToDegrees = Convert.ToByte(UserAttitudeToDegrees.Text);

                    switch ( RxPacketTag ) {
                    case UAVXFlightPacketTag:

                        for (i = 2; i < (NoOfFlagBytes+2); i++)
                            Flags[i-2] = ExtractByte(ref UAVXPacket, i);

                        if ((Flags[0] & 0x01) != 0) 
                            RTHAltHoldBox.BackColor = System.Drawing.Color.Green;
                        else
                            RTHAltHoldBox.BackColor = System.Drawing.Color.Red;
                        if ((Flags[0] & 0x02) != 0)
                            TurnToWPBox.BackColor = System.Drawing.Color.Green;
                        else
                            TurnToWPBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[0] & 0x04) != 0)
                            MotorsArmedBox.BackColor = System.Drawing.Color.Green;
                        else
                            MotorsArmedBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[0] & 0x08) != 0)
                            LostModelBox.BackColor = System.Drawing.Color.Red;
                        else
                            LostModelBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[0] & 0x10) != 0)
                            NearLevelBox.BackColor = System.Drawing.Color.Green;
                        else
                            NearLevelBox.BackColor = System.Drawing.Color.LightSteelBlue;
                        if ((Flags[0] & 0x20) != 0)
                        {
                            LowBatteryBox.BackColor = System.Drawing.Color.Red;
                            BatteryVolts.BackColor = System.Drawing.Color.Red;
                        }
                        else
                        {
                            LowBatteryBox.BackColor = FlagsGroupBox.BackColor;
                            BatteryVolts.BackColor = BatteryGroupBox.BackColor;
                        }
                        if ((Flags[0] & 0x40) != 0)
                            GPSValidBox.BackColor = System.Drawing.Color.Green;
                        else
                            GPSValidBox.BackColor = System.Drawing.Color.Red;
                        if ((Flags[0] & 0x80) != 0)
                            NavValidBox.BackColor = System.Drawing.Color.Green;
                        else
                            NavValidBox.BackColor = System.Drawing.Color.Red;

                        if ((Flags[1] & 0x01) != 0)
                            BaroFailBox.BackColor = System.Drawing.Color.Red;
                        else
                            BaroFailBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[1] & 0x02) != 0)
                            AccFailBox.BackColor = System.Drawing.Color.Red;
                        else
                            AccFailBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[1] & 0x04) != 0)
                            CompassFailBox.BackColor = System.Drawing.Color.Red;
                        else
                            CompassFailBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[1] & 0x08) != 0)
                            GPSFailBox.BackColor = System.Drawing.Color.Red;
                        else
                            GPSFailBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[1] & 0x10) != 0)
                            AttitudeHoldBox.BackColor = System.Drawing.Color.Green;
                        else
                            AttitudeHoldBox.BackColor = System.Drawing.Color.LightSteelBlue;
                        if ((Flags[1] & 0x20) != 0)
                            ThrottleMovingBox.BackColor = System.Drawing.Color.LightSteelBlue;
                        else
                            ThrottleMovingBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[1] & 0x40) != 0)
                            HoldingAltBox.BackColor = System.Drawing.Color.Green;
                        else
                            HoldingAltBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[1] & 0x80) != 0)
                            NavigateBox.BackColor = System.Drawing.Color.Green;
                        else
                            NavigateBox.BackColor = FlagsGroupBox.BackColor;
                 
                        if ((Flags[2] & 0x01) != 0)
                            ReturnHomeBox.BackColor = System.Drawing.Color.Green;
                        else
                            ReturnHomeBox.BackColor = FlagsGroupBox.BackColor;

                        if ((Flags[2] & 0x02) != 0)
                            ProximityBox.BackColor = System.Drawing.Color.Green;
                        else
                            ProximityBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[2] & 0x04) != 0)
                            CloseProximityBox.BackColor = System.Drawing.Color.Green;
                        else
                            CloseProximityBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[2] & 0x08) != 0)
                            UsingGPSAltBox.BackColor = System.Drawing.Color.Green;
                        else
                            UsingGPSAltBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[2] & 0x10) != 0)
                            UseRTHAutoDescendBox.BackColor = System.Drawing.Color.Green;
                        else
                            UseRTHAutoDescendBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[2] & 0x20) != 0)
                            BaroAltValidBox.BackColor = System.Drawing.Color.Green;
                        else
                            BaroAltValidBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[2] & 0x40) != 0)
                        {
                            RangefinderAltValidBox.BackColor = System.Drawing.Color.Green;
                            RangefinderAltitude.BackColor = AltitudeGroupBox.BackColor;
                            RangefinderROC.BackColor = AltitudeGroupBox.BackColor;
                        }
                        else
                        {
                            RangefinderAltValidBox.BackColor = FlagsGroupBox.BackColor;
                            RangefinderAltitude.BackColor = System.Drawing.Color.Orange;
                            RangefinderROC.BackColor = System.Drawing.Color.Orange;
                        }
                        if ((Flags[2] & 0x80) != 0)
                            UsingRangefinderBox.BackColor = System.Drawing.Color.Green;
                        else
                            UsingRangefinderBox.BackColor = FlagsGroupBox.BackColor;
    
                        StateT = ExtractByte(ref UAVXPacket, 8);
                        switch ( StateT ){
                            case 0: FlightState.Text = "Starting"; break;
                            case 1: FlightState.Text = "Landing"; break;
                            case 2: FlightState.Text = "Landed"; break;
                            case 3: FlightState.Text = "Flying"; break;
                            default: FlightState.Text = "Unknown"; break;
                        } // switch

                        BatteryVoltsT =  ExtractShort(ref UAVXPacket, 9);
                        BatteryCurrentT = ExtractShort(ref UAVXPacket, 11);
                        BatteryChargeT = ExtractShort(ref UAVXPacket, 13);
                        RCGlitchesT = ExtractShort(ref UAVXPacket, 15);
                        DesiredThrottleT = ExtractShort(ref UAVXPacket, 17);
                        DesiredRollT = ExtractShort(ref UAVXPacket, 19);
                        DesiredPitchT = ExtractShort(ref UAVXPacket, 21);
                        DesiredYawT = ExtractShort(ref UAVXPacket, 23);
                        RollRateT = ExtractShort(ref UAVXPacket, 25);
                        PitchRateT = ExtractShort(ref UAVXPacket, 27);
                        YawRateT = ExtractShort(ref UAVXPacket, 29);
                        RollSumT = ExtractShort(ref UAVXPacket, 31);
                        PitchSumT = ExtractShort(ref UAVXPacket, 33);
                        YawSumT = ExtractShort(ref UAVXPacket, 35);
                        LRAccT = ExtractShort(ref UAVXPacket, 37);
                        FBAccT = ExtractShort(ref UAVXPacket, 39);
                        DUAccT = ExtractShort(ref UAVXPacket, 41);
                        LRCompT = ExtractSignedByte(ref UAVXPacket, 43);
                        FBCompT = ExtractSignedByte(ref UAVXPacket, 44);
                        DUCompT = ExtractSignedByte(ref UAVXPacket, 45);
                        AltCompT = ExtractSignedByte(ref UAVXPacket, 46);

                        BatteryVolts.Text = string.Format("{0:n1}", ((float)BatteryVoltsT / 37.95)); 
                        BatteryCurrent.Text = string.Format("{0:n1}", (float)BatteryCurrentT * 0.1);
                        BatteryCharge.Text = string.Format("{0:n0}", (float)BatteryChargeT * BatteryChargeScale);
                        RCGlitches.Text = string.Format("{0:n0}", RCGlitchesT); 
                        DesiredThrottle.Text = string.Format("{0:n0}", ((float)DesiredThrottleT * 100.0)/ RCMaximum);
                        DesiredRoll.Text = string.Format("{0:n0}", ((float)DesiredRollT * 200.0)/ RCMaximum); 
                        DesiredPitch.Text = string.Format("{0:n0}", ((float)DesiredPitchT * 200.0)/ RCMaximum);
                        DesiredYaw.Text = string.Format("{0:n0}", ((float)DesiredYawT * 200.0) / RCMaximum); 
                        RollRate.Text = string.Format("{0:n0}", RollRateT); 
                        PitchRate.Text = string.Format("{0:n0}", PitchRateT); 
                        YawRate.Text = string.Format("{0:n0}", YawRateT);
                        RollSum.Text = string.Format("{0:n0}", RollSumT / AttitudeToDegrees);
                        PitchSum.Text = string.Format("{0:n0}", PitchSumT / AttitudeToDegrees);
                        YawSum.Text = string.Format("{0:n0}", YawSumT / AttitudeToDegrees); 
                        LRAcc.Text = string.Format("{0:n2}", (float)LRAccT / 1024.0);
                        FBAcc.Text = string.Format("{0:n2}", (float)FBAccT / 1024.0);
                        DUAcc.Text = string.Format("{0:n2}", (float)DUAccT / 1024.0);
                        LRComp.Text = string.Format("{0:n0}", LRCompT );
                        FBComp.Text = string.Format("{0:n0}", FBCompT );
                        DUComp.Text = string.Format("{0:n0}", DUCompT );
                        AltComp.Text = string.Format("{0:n0}", AltCompT );

                        attitudeIndicatorInstrumentControl1.SetAttitudeIndicatorParameters(
                            -PitchSumT / AttitudeToDegrees,
                            RollSumT / AttitudeToDegrees
                            );
                           
                        break;
                    case UAVXNavPacketTag:
                        NavStateT = ExtractByte(ref UAVXPacket, 2);
                        switch (NavStateT)
                        {
                            case 0: NavState.Text = "Holding";
                                NavState.BackColor = System.Drawing.Color.LightSteelBlue;
                                break;
                            case 1: NavState.Text = "Returning";
                                NavState.BackColor = System.Drawing.Color.Lime;
                                break;
                            case 2: NavState.Text = "Home";
                                NavState.BackColor = System.Drawing.Color.Green;
                                break;
                            case 3: NavState.Text = "Descending";
                                NavState.BackColor = System.Drawing.Color.Orange;
                                break;
                            case 4: NavState.Text = "Touchdown!";
                                NavState.BackColor = System.Drawing.Color.RosyBrown;
                                break;
                            case 5: NavState.Text = "Navigating";
                                NavState.BackColor = System.Drawing.Color.Silver;
                                break;
                            case 6: NavState.Text = "Loitering";
                                NavState.BackColor = System.Drawing.Color.Gold;
                                break;
                            case 7: NavState.Text = "Terminating";
                                NavState.BackColor = System.Drawing.Color.Red;
                                break;
                            default: NavState.Text = "Unknown"; break;
                        } // switch

                        FailStateT = ExtractByte(ref UAVXPacket, 3);
                        switch (FailStateT)
                        {
                            case 0: FailState.Text = "Monitor Rx";
                                FailState.BackColor = System.Drawing.Color.Green;
                                break;
                            case 1: FailState.Text = "Aborting";
                                FailState.BackColor = System.Drawing.Color.Orange;
                                break;
                            case 2: FailState.Text = "Returning";
                                FailState.BackColor = System.Drawing.Color.Orange;
                                break;
                            case 3: FailState.Text = "Terminating";
                                FailState.BackColor = System.Drawing.Color.Red;
                                break;
                            default: FailState.Text = "Unknown"; break;
                        } // switch
                            
                        GPSNoOfSatsT = ExtractByte(ref UAVXPacket, 4);
                        GPSFixT = ExtractByte(ref UAVXPacket, 5);
                        CurrWPT = ExtractByte(ref UAVXPacket, 6);
                        BaroROCT = ExtractShort(ref UAVXPacket, 7);
                        RelBaroAltitudeT = ExtractInt24(ref UAVXPacket, 9);
                        RangefinderROCT = ExtractShort(ref UAVXPacket, 12);
                        RangefinderAltitudeT = ExtractShort(ref UAVXPacket, 14);
                        GPSHDiluteT = ExtractShort(ref UAVXPacket, 16);
                        HeadingT = ExtractShort(ref UAVXPacket, 18);
                        DesiredCourseT = ExtractShort(ref UAVXPacket, 20);
                        GPSVelT = ExtractShort(ref UAVXPacket, 22);
                        GPSROCT = ExtractShort(ref UAVXPacket, 24);                 
                        GPSRelAltitudeT = ExtractInt24(ref UAVXPacket, 26); 
                        GPSLatitudeT = ExtractInt(ref UAVXPacket, 29);
                        GPSLongitudeT = ExtractInt(ref UAVXPacket, 33);
                        DesiredAltitudeT = ExtractInt24(ref UAVXPacket, 37);
                        DesiredLatitudeT = ExtractInt(ref UAVXPacket, 40);
                        DesiredLongitudeT = ExtractInt(ref UAVXPacket, 44);
                        NavStateTimeoutT = ExtractInt24(ref UAVXPacket, 48);

                        if ( FirstGPSCoordinates && ((Flags[0] & 0x80) != 0))
                        {
                            FirstGPSCoordinates = false;
                            OriginLatitude = GPSLatitudeT;
                            OriginLongitude = GPSLongitudeT;
                        }

                        GPSNoOfSats.Text = string.Format("{0:n0}", GPSNoOfSatsT);
                        if ( GPSNoOfSatsT < 6 )
                            GPSNoOfSats.BackColor = System.Drawing.Color.Orange;
                        else
                            GPSNoOfSats.BackColor = GPSStatBox.BackColor;
                        
                        GPSFix.Text = string.Format("{0:n0}", GPSFixT);
                        if ( GPSFixT < 2 )
                            GPSFix.BackColor = System.Drawing.Color.Orange;
                        else
                            GPSFix.BackColor = GPSStatBox.BackColor;

                        GPSHDilute.Text = string.Format("{0:n2}", (float)GPSHDiluteT * 0.01);
                        if (GPSHDiluteT > 130)
                            GPSHDilute.BackColor = System.Drawing.Color.Orange;
                        else
                            GPSHDilute.BackColor = GPSStatBox.BackColor;

                        CurrWP.Text = string.Format("{0:n0}", CurrWPT );
                        //pad1.Text = string.Format("{0:n0}", ExtractByte(ref UAVXPacket, 7));

                        BaroROC.Text = string.Format("{0:n2}", (float)BaroROCT * 0.01);
                        RelBaroAltitude.Text = string.Format("{0:n2}", (float)RelBaroAltitudeT * 0.01);

                        if (RelBaroAltitudeT > ( MaximumAltitudeLimit*100) )
                            RelBaroAltitude.BackColor = System.Drawing.Color.Orange;
                        else
                            RelBaroAltitude.BackColor = AltitudeGroupBox.BackColor;

                        if ( BaroROCT < -100 ) // 1M/S
                            BaroROC.BackColor = System.Drawing.Color.Orange;
                        else
                            BaroROC.BackColor = AltitudeGroupBox.BackColor;

                        RangefinderROC.Text = string.Format("{0:n1}", (float)RangefinderROCT * 0.01);
                        RangefinderAltitude.Text = string.Format("{0:n2}", (float)RangefinderAltitudeT * 0.01 );

                        Heading.Text = string.Format("{0:n0}", ((int)HeadingT * 180) / 3142);
                        
                        if ((Flags[2] & 0x80) != 0)
                        {
                            CurrAlt = RangefinderAltitudeT;
                            AltitudeSource.Text = "Rangefinder";
                        }
                        else
                            if ((Flags[2] & 0x08) != 0)
                            {
                                CurrAlt = GPSRelAltitudeT;
                                AltitudeSource.Text = "GPS";
                            }
                            else
                            {
                                CurrAlt = RelBaroAltitudeT;
                                AltitudeSource.Text = "Barometer";
                            }

                        CurrentAltitude.Text = string.Format("{0:n0}", (float)CurrAlt * 0.01);
                        if ((CurrAlt * 0.01) > MaximumAltitudeLimit)
                            CurrentAltitude.BackColor = System.Drawing.Color.Orange;
                        else
                            CurrentAltitude.BackColor = NavGroupBox.BackColor;

                        AltError = CurrAlt - DesiredAltitudeT;
                        AltitudeError.Text = string.Format("{0:n1}", (float)AltError * 0.01);

                        GPSVel.Text = string.Format("{0:n1}", (double)GPSVelT * 0.1); // dM/Sec
                        GPSROC.Text = string.Format("{0:n1}", (float)GPSROCT * 0.01);
                        GPSRelAltitude.Text = string.Format("{0:n2}", (double)GPSRelAltitudeT * 0.01);
                        GPSLongitude.Text = string.Format("{0:n6}", (double)GPSLongitudeT / 6000000.0);
                        GPSLatitude.Text = string.Format("{0:n6}", (double)GPSLatitudeT / 6000000.0);

                        if ((Flags[0] & 0x40) != 0) // GPSValid
                        {
                            GPSVel.BackColor = NavGroupBox.BackColor;
                            GPSROC.BackColor = NavGroupBox.BackColor;
                            GPSRelAltitude.BackColor = NavGroupBox.BackColor;
                            GPSLongitude.BackColor = NavGroupBox.BackColor;
                            GPSLatitude.BackColor = NavGroupBox.BackColor;
                            WayHeading.BackColor = NavGroupBox.BackColor;
                            DistanceToDesired.BackColor = NavGroupBox.BackColor;

                            WayHeading.Text = string.Format("{0:n0}", (DesiredCourseT * 180) / 3142);

                            LongitudeCorrection = Math.Cos(Math.PI/180.0 * (DesiredLatitudeT + GPSLatitudeT) / 12000000.0 ); 
                        
                            NorthDiff = (double)(DesiredLatitudeT - GPSLatitudeT); // scale up to decimetres after conversion
                            EastDiff = (double)(DesiredLongitudeT - GPSLongitudeT) * LongitudeCorrection;

                            Distance = ConvertGPSToM(Math.Sqrt(NorthDiff * NorthDiff + EastDiff * EastDiff));
                            DistanceToDesired.Text = string.Format("{0:n1}", Distance);
                        }
                        else
                        {

                            GPSVel.BackColor = System.Drawing.Color.Orange;
                            GPSROC.BackColor = System.Drawing.Color.Orange;
                            GPSRelAltitude.BackColor = System.Drawing.Color.Orange;
                            GPSLongitude.BackColor = System.Drawing.Color.Orange;
                            GPSLatitude.BackColor = System.Drawing.Color.Orange;
                            WayHeading.BackColor = System.Drawing.Color.Orange;
                            DistanceToDesired.BackColor = System.Drawing.Color.Orange;

                            WayHeading.Text = "?";
                            DistanceToDesired.Text = "?";       
                        }

                        if ( !FirstGPSCoordinates ) 
                        {
                            WhereBearing.BackColor = NavGroupBox.BackColor;
                            WhereDistance.BackColor = NavGroupBox.BackColor;

                            NorthDiff = GPSLatitudeT - OriginLatitude;

                            LongitudeCorrection = Math.Cos(Math.PI / 180.0 * (GPSLatitudeT + OriginLatitude) / 12000000.0);

                            EastDiff = (GPSLongitudeT - OriginLongitude) * LongitudeCorrection;
                            WhereDirection = Math.Atan2(-EastDiff, NorthDiff) * 180.0 / Math.PI;
                            while (WhereDirection < 0)
                                WhereDirection += 360.0;
                            WhereBearing.Text = string.Format("{0:n0}", WhereDirection);
                            Distance = ConvertGPSToM(Math.Sqrt(NorthDiff * NorthDiff + EastDiff * EastDiff));
                            WhereDistance.Text = string.Format("{0:n0}", Distance);
                            if ( Distance > MaximumRangeLimit )
                                WhereDistance.BackColor = System.Drawing.Color.Orange;
                            else
                                WhereDistance.BackColor = NavGroupBox.BackColor;
                        }
                        else
                        {
                            WhereBearing.BackColor = System.Drawing.Color.Orange;
                            WhereDistance.BackColor = System.Drawing.Color.Orange;

                            WhereBearing.Text = "?";
                            WhereDistance.Text = "?";
                        }

                        if (NavStateTimeoutT >= 0)
                            NavStateTimeout.Text = string.Format("{0:n0}", NavStateTimeoutT / 1000);
                        else
                            NavStateTimeout.Text = " ";

                        headingIndicatorInstrumentControl1.SetHeadingIndicatorParameters(
                            ((HeadingT * 180) / 3142));
 
                        break;
                    default: break;
                    } // switch
                }
            }
        }

        private void UAVXCloseTelemetry()
        {
            if (serialPort1.IsOpen) serialPort1.Close();
        }

        //----------------------------------------------------------------------- 

        public void Zero(ref byte [] a, short len)
        {
            for (int s = 0; s < len; s++)
                a[s] = 0;
        }

        public byte ExtractByte(ref byte [] a, short p)
        {
            return a[p];
        }

        public short ExtractSignedByte(ref byte[] a, short p)
        {
            short temp;

            temp = a[p];
            if ( temp > 127 )
                temp -= 256;

            return temp;
        }

        public short ExtractShort(ref byte[] a, short p)
        {        
            short temp;
            temp = (short)(a[p + 1] << 8);
            temp |= (short)a[p];
   
            return temp;
        }

        public int ExtractInt24(ref byte[] a, short p)
        {
            int temp;

            temp = ((int)a[p + 2] << 24);
            temp |= ((int)a[p + 1] << 16);
            temp |= (int)a[p] << 8;
            temp /= 256;
            return temp;
        }

        public int ExtractInt(ref byte[] a, short p)
        {
            int temp;

            temp = (int)(a[p + 3] << 24);
            temp |= ((int)a[p + 2] << 16);
            temp |= ((int)a[p + 1] << 8);
            temp |= (int)a[p];
            return temp;
        }

        public double ConvertGPSToM(double c)
        {	// converts to Metres
            // approximately 1.8553257183 cm per LSB at the Equator
            // conversion max is 21Km
            return (c * 0.018553257183);
        }

    
    }
}