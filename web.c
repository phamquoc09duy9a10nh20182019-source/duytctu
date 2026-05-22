using DocumentFormat.OpenXml.Vml;
using OfficeOpenXml;
using OfficeOpenXml.Style;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Xml.Serialization;
using ZedGraph;
using Microsoft.VisualBasic;
namespace App_1
{
    public partial class Form1 : Form
    {
        Stopwatch stopwatch = new Stopwatch();
        Form4 manHinhPhu = new Form4();
        public Form1()
        {
            InitializeComponent();
            string[] Baudrate = { "300", "600", "750", "1200", "2400", "4800", "9600", "19200", "31250", "38400", "57600", "74880", "115200", "230400", "250000", "460800", "500000", "921600", "1000000", "2000000", "5000000" };
            baudrate.Items.AddRange(Baudrate);
            Control.CheckForIllegalCrossThreadCalls = false;
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            port.DataSource = SerialPort.GetPortNames();
            baudrate.Text = "115200";


        }

        private void run_Click(object sender, EventArgs e)
        {
            if (!serCom.IsOpen)
            {
                MessageBox.Show("Bạn chưa kết nối cổng serial ^^", "Cảnh báo", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
            else
            {
                byte[] data = new byte[] {
                    0xAA, 
                    (byte)'C' 
                };
                serCom.Write(data, 0, data.Length);
            }

        }

        private void stop_Click(object sender, EventArgs e)
        {
            if (!serCom.IsOpen)
            {
                MessageBox.Show("Bạn chưa kết nối cổng serial ^^", "Cảnh báo", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
            else
            {
                byte[] data = new byte[] {
                    0xAA,
                    (byte)'D'
                };
                serCom.Write(data, 0, data.Length);
            }
        }

        private void time_Click(object sender, EventArgs e)
        {
            if (!serCom.IsOpen)
            {
                MessageBox.Show("Bạn chưa kết nối cổng serial ^^", "Cảnh báo", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
            else
            {
                stopwatch.Restart();
                byte[] data = new byte[] {
                    0xAA,
                    (byte)'T'
                };
                serCom.Write(data, 0, data.Length);
            }
        }
private void reset_Click(object sender, EventArgs e)
        {
            if (!serCom.IsOpen)
            {
                MessageBox.Show("Bạn chưa kết nối cổng serial ^^", "Lỗi", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
else
            {
                byte[] data = new byte[] {
                    0xAA,
                    (byte)'R'
                };
                serCom.Write(data, 0, data.Length);
            }
        }

        private void connect_Click(object sender, EventArgs e)
        {    // 1. Kiểm tra đã chọn cổng COM chưa
            if (string.IsNullOrWhiteSpace(port.Text))
            {
                MessageBox.Show("Bạn chưa chọn cổng COM ^^", "Cảnh báo", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return; // dừng hàm, không mở COM
            }

            // 2. Kiểm tra đã chọn Baud rate chưa
            if (string.IsNullOrWhiteSpace(baudrate.Text))
            {
                MessageBox.Show("Bạn chưa chọn Baudrate ^^", "Cảnh báo", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }
            if (!serCom.IsOpen)
            {
                connect.Text = "Đã kết nối";
                serCom.PortName = port.Text;
                serCom.BaudRate = Convert.ToInt32(baudrate.Text);

                serCom.Open();
                txtSet1.Text = "0";
                txtSet2.Text = "0";
                txtSet3.Text = "0";
            }
            else
            {
                connect.Text = "Ngắt kết nối";
                serCom.Close();
            }
        }

        private void exits_Click(object sender, EventArgs e)
        {
            Application.Exit();
        }
        List<byte> rxBuffer = new List<byte>();
        private void serCom_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            int bytes = serCom.BytesToRead;
            byte[] temp = new byte[bytes];
            serCom.Read(temp, 0, bytes);
            rxBuffer.AddRange(temp);
            // Xử lý frame
            while (rxBuffer.Count >= 2)
            {
                // Tìm header
                if (rxBuffer[0] != 0xAA)
                {
                    rxBuffer.RemoveAt(0);
                    continue;
                }
                byte cmd = rxBuffer[1];
                // Kiểm tra 
                if (cmd == (byte)'P')
                {
                    if (rxBuffer.Count < 8) return;

                    short p1 = (short)((rxBuffer[2] << 8) | rxBuffer[3]);
                    short p2 = (short)((rxBuffer[4] << 8) | rxBuffer[5]);
                    short p3 = (short)((rxBuffer[6] << 8) | rxBuffer[7]);
                    rxBuffer.RemoveRange(0, 8);
                    // Đổi xung sang góc
                    double goc1 = p1 * 360.0 / 8192.0;
                    double goc2 = p2 * 360.0 / 8192.0;
                    double goc3 = p3 * 360.0 / 8192.0;

                    txtReal1.Text = goc1 .ToString("F2");
                    txtReal2.Text = goc2 .ToString("F2");
                    txtReal3.Text = goc3 .ToString("F2");
manHinhPhu.ThemDuLieu(goc1.ToString("F2"), goc2.ToString("F2"), goc3.ToString("F2"));

                }
                else if (cmd == (byte)'T')
                {
                    if (rxBuffer.Count < 8) return;

                    short t1 = (short)((rxBuffer[2] << 8) | rxBuffer[3]);
                    short t2 = (short)((rxBuffer[4] << 8) | rxBuffer[5]);
                    short t3 = (short)((rxBuffer[6] << 8) | rxBuffer[7]);

                    rxBuffer.RemoveRange(0, 8);
                    // Đổi xung sang góc
                    double target_goc1 = t1 * 360.0 / 8192.0;
                    double target_goc2 = t2 * 360.0 / 8192.0;
                    double target_goc3 = t3 * 360.0 / 8192.0;
                    txtSet1.Text = target_goc1 .ToString("F2");
                    txtSet2.Text = target_goc2 .ToString("F2");
                    txtSet3.Text = target_goc3 .ToString("F2");

                }
                else if (cmd == (byte)'D')
                {
                    if (rxBuffer.Count < 2) return;

                    rxBuffer.RemoveRange(0, 2);


                    stopwatch.Stop();
                    txtTime.Text = $"Thời gian: {stopwatch.Elapsed.TotalMilliseconds} ms";

                }

                // Cảnh báo
                else if (cmd == (byte)'W')
                {
                    if (rxBuffer.Count < 2) return;

                    rxBuffer.RemoveRange(0, 2);


                    MessageBox.Show("Đã vượt ra khỏi workspaces ^^", "Cảnh báo", MessageBoxButtons.OK, MessageBoxIcon.Warning, MessageBoxDefaultButton.Button1, MessageBoxOptions.ServiceNotification);

                }

                else
                {
                    // Không hợp lệ thì bỏ header
                    rxBuffer.RemoveAt(0);
                }
            }

              
        }
        private void TinhToaDo(
    double theta1,
    double theta2,
    double theta3,
    out double toado_X,
    out double toado_Y)
        {
            double offset_1 = -19.49;
            double offset_2 = 100.51;
            double offset_3 = -139.49;
            double l = 20.0;
            double r = 5.0 / Math.Sqrt(3.0);
            double th1 = (theta1 + offset_1) * Math.PI / 180.0;
            double th2 = (theta2 + offset_2) * Math.PI / 180.0;
            double th3 = (theta3 + offset_3) * Math.PI / 180.0;
            double Px = l * Math.Cos(th1) + l * Math.Cos(th1 + th2) + r * Math.Cos(th1 + th2 + th3 + Math.PI / 6.0);
            double Py = l * Math.Sin(th1) + l * Math.Sin(th1 + th2) + r * Math.Sin(th1 + th2 + th3 + Math.PI / 6.0);
            toado_X = Px;
            toado_Y = Py;
        }
        private void sendPos_Click(object sender, EventArgs e)
        {
            if (!serCom.IsOpen)
            {
                MessageBox.Show("Bạn chưa kết nối cổng serial ^^", "Lỗi", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
else if (string.IsNullOrWhiteSpace(txtSend1.Text) || string.IsNullOrWhiteSpace(txtSend2.Text) ||string.IsNullOrWhiteSpace(txtSend3.Text))
            {
                MessageBox.Show("Bạn chưa nhập đủ dữ liệu ^^", "Cảnh báo", MessageBoxButtons.OK, MessageBoxIcon.Warning, MessageBoxDefaultButton.Button1, MessageBoxOptions.ServiceNotification);
                return;
            }
            else 
            {
                double Goc_1 = double.Parse(txtSend1.Text);
                double Goc_2 = double.Parse(txtSend2.Text);
                double Goc_3 = double.Parse(txtSend3.Text);
                TinhToaDo(Goc_1, Goc_2, Goc_3, out double toado_x, out double toado_y);
                txtSetX.Text = toado_x.ToString("F2");
                txtSetY.Text = toado_y.ToString("F2");
            }
        }
        private void sendXY_Click(object sender, EventArgs e)
        {
            if (!serCom.IsOpen)
            {
                MessageBox.Show("Bạn chưa kết nối cổng serial ^^", "Lỗi", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
            else if (string.IsNullOrWhiteSpace(txtSendX.Text) || string.IsNullOrWhiteSpace(txtSendY.Text))
            {
                MessageBox.Show("Bạn chưa nhập đủ dữ liệu ^^", "Cảnh báo", MessageBoxButtons.OK, MessageBoxIcon.Warning, MessageBoxDefaultButton.Button1, MessageBoxOptions.ServiceNotification);
                return;
            }
            else
            {
                double X = double.Parse(txtSendX.Text);
                double Y = double.Parse(txtSendY.Text);
                double X_1 = X * 100;
                double Y_1 = Y * 100;
                short x = (short)X_1;
                short y = (short)Y_1;

                byte[] XY = new byte[]
                {
                    0xAA,
                    (byte)'S',
                    (byte)(x >> 8), (byte)(x & 0xFF),
                    (byte)(y >> 8), (byte)(y & 0xFF),
                };

                serCom.Write(XY, 0, XY.Length);
            }
        }      

        
        private void txt_TextChanged(object sender, EventArgs e)
        {

        }

        private void port_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void txtSet1_TextChanged(object sender, EventArgs e)
        {

        }

        private void txtSend1_TextChanged(object sender, EventArgs e)
        {

        }



        private void Excel_Click(object sender, EventArgs e)
        {
            if (!manHinhPhu.Visible)
            {
                manHinhPhu.Show(); // Hiện Form4
            }

            manHinhPhu.Activate(); // Đưa Form4 lên phía trên cùng của màn hình
            manHinhPhu.WindowState = FormWindowState.Normal; // Đảm bảo nó không bị thu nhỏ dưới Taskbar
        }

        private void label7_Click(object sender, EventArgs e)
        {

        }

      
    }
}