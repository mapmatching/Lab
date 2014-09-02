using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml;
using System.IO;

namespace OSMProcess
{
    class Program
    {
        class Node
        {
            public Node(double lat, double lon)
            {
                this.lat = lat;
                this.lon = lon;
                this.newId = -1;
            }
            public double lat;
            public double lon;
            public int newId;
        }

        class Way
        {

        }
        static void Main(string[] args)
        {
            XmlDocument dom = new XmlDocument();
            //dom.Load(@"D:\trajectory\singapore_data\singapore_map\map_new_in 2014-05-27.xml");
           // dom.Load(@"D:\trajectory\beijing_data\beijing_map\map_new_in 2014-06-26.xml");
            dom.Load(@"C:\Users\wuhao\Desktop\athens.xml");
            Console.WriteLine("xml加载完毕！");
            StreamWriter edgeSw = new StreamWriter("edgeOSM.txt");
            StreamWriter nodeSw = new StreamWriter("nodeOSM.txt");
            Dictionary<long, Node> nodeDic = new Dictionary<long,Node>();
            List<Node> extractedNodes = new List<Node>();
            int nodeCount = 0;
            int wayCount = 0;

            foreach(XmlElement entry in dom.DocumentElement.ChildNodes)
            {
                if(entry.Name.Equals("node"))
                {
                    long oldId = long.Parse(entry.GetAttribute("id"));
                    double lat = double.Parse(entry.GetAttribute("lat"));
                    double lon = double.Parse(entry.GetAttribute("lon"));
                    Node node = new Node(lat, lon);                   
                    nodeDic.Add(oldId, node);
                }
                if (entry.Name.Equals("way"))
                {
                    List<long> figure = new List<long>();
                    bool isHighWay = false;
                    bool isOneWay = false;
                    bool isBorder = false;
                    foreach (XmlElement childNode in entry.ChildNodes)
                    {
                        if (childNode.Name.Equals("nd"))
                        {
                            figure.Add(long.Parse(childNode.GetAttribute("ref")));
                        }
                        if (childNode.Name.Equals("tag"))
                        {
                            if (childNode.GetAttribute("k").Equals("highway"))
                                isHighWay = true;
                            if (childNode.GetAttribute("k").Equals("oneway"))
                                isOneWay = true;
                            if (childNode.GetAttribute("k").Equals("boundary") && childNode.GetAttribute("v").Equals("administrative"))
                                isBorder = true;
                        }
                    }
                    if (isBorder == false)
                        continue;
                   // if (isHighWay == false)
                    //    continue;
                    //处理头
                    Node fromNode;
                    if (nodeDic.TryGetValue(figure[0], out fromNode))
                    {
                        if (fromNode.newId == -1)
                        {
                            fromNode.newId = nodeCount++;
                            extractedNodes.Add(fromNode);
                        }
                        edgeSw.Write(wayCount++ + "\t" + fromNode.newId);
                    }
                    else
                    {
                        Console.WriteLine("不存在的键值！");
                        Console.ReadKey(true);
                    }
                    //处理尾
                    Node toNode;
                    if (nodeDic.TryGetValue(figure[figure.Count-1], out toNode))
                    {
                        if (toNode.newId == -1)
                        {
                            toNode.newId = nodeCount++;
                            extractedNodes.Add(toNode);
                        }
                        edgeSw.Write("\t" + toNode.newId);
                    }
                    else
                    {
                        Console.WriteLine("不存在的键值！");
                        Console.ReadKey(true);
                    }
                    //输出路形
                    edgeSw.Write("\t" + figure.Count);
                    for (int i = 0; i < figure.Count; i++)
                    {
                        edgeSw.Write("\t" + nodeDic[figure[i]].lat + "\t" + nodeDic[figure[i]].lon);
                    }
                    edgeSw.Write("\n");

                    //双向路反向输出
                    if (isOneWay)   
                        continue;
                    edgeSw.Write(wayCount++ + "\t" + toNode.newId + "\t" + fromNode.newId + "\t" + figure.Count);
                    for (int i = figure.Count-1; i >= 0; i--)
                    {
                        edgeSw.Write("\t" + nodeDic[figure[i]].lat + "\t" + nodeDic[figure[i]].lon);
                    }
                    edgeSw.Write("\n");
                }
            }
            //输出ndoes
            for (int i = 0; i < extractedNodes.Count; i++)
            {
                nodeSw.Write(i + "\t" + extractedNodes[i].lat + "\t" + extractedNodes[i].lon + "\n");
            }
            edgeSw.Close();
            nodeSw.Close();
        }
    }
}
