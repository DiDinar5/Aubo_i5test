using System.Windows;
using System.Windows.Input;

namespace MoveObjectWithMouse
{
    public partial class MainWindow : Window
    {
        Point oldPosition = new Point();
        public MainWindow()
        {   
            InitializeComponent();
            RobotAubo.RunRobot();
        }

        private void MouseMoveHandler(object sender, MouseEventArgs e)
        {
            // Get the x and y coordinates of the mouse pointer.
            var position = e.GetPosition(this);
            
            var pX = position.X;
            var pY = position.Y;

            // Sets the Height/Width of the circle to the mouse coordinates.
            ellipse.Width = pX;
            ellipse.Height = pY;
            //position текущая, oldPosition предыдущая
            RobotAubo.relativeOffsetExample(position.X - oldPosition.X, position.Y - oldPosition.Y);
            oldPosition = position;
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            RobotAubo.StopRobot();
            this.Close();
        }
    }
}