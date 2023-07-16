using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MoveObjectWithMouse
{
    public  class CustomLog
    {
        public static void Log(string message)
        {
            File.AppendAllLines($"log.txt",new List<string>{$"{DateTime.Now}: {message}"});
        }
    }
}
