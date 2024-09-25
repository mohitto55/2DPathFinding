using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Tilemaps;
using System;
using TMPro;
public class Node
{
    public Vector2 Pos;
    public List<ConnectNode> ConnectNodesList = new List<ConnectNode>();
    public bool NodeContains(Node node) {
        for(int i = 0; i < ConnectNodesList.Count; i++) {
            if (ConnectNodesList[i].node == node)
                return true;
        }
        return false;
    }
    public ConnectNode FindConnectNode(Node node) {
        for(int i = 0; i < ConnectNodesList.Count; i++) {
            if(ConnectNodesList[i].node == node) {
                return ConnectNodesList[i];
            }
        }
        return new ConnectNode();
    }
    public Node(Vector2 pos)
    {
        Pos = pos;
        ConnectNodesList = new List<ConnectNode>();
    }

    public struct ConnectNode {
        public Node node;
        public float distance;
        public ConnectNodeType nodeType;
        public ConnectNode(Node node,float distance, ConnectNodeType type) {
            this.node = node;
            this.distance = distance;
            this.nodeType = type;
        }
    }
    public enum ConnectNodeType {
        Up, Down, Straight, Cliff
    }
}
public class PlatformPathFinding : SingletonBehaviour<PlatformPathFinding>
{
    [SerializeField] LayerMask platformLayerMask;
    [SerializeField] BoxZone2D boxZone;
    List<Node> NodeList = new List<Node>();
    List<Vector2> TilesPos = new List<Vector2>();
    Dictionary<Vector2, TileBase> MapTiles = new Dictionary<Vector2, TileBase>();
    public float JumpPoawer;
    public float Gravity;
    public float Velocity;
    public int CliffDst = 4;
    public override void Awake()
    {
        base.Awake();
        CreateMap();
        CreateNode();
    }
    void CreateMap()
    {
        List<Vector2> allCell = boxZone.GetAllCells(1, 1);
        int platformLayer = platformLayerMask;
        for (int i = 0; i < allCell.Count; i++) {
            RaycastHit2D hit = Physics2D.Raycast(allCell[i], Vector2.down, 0.501f, platformLayer);
            Collider2D co2D = Physics2D.OverlapPoint(allCell[i], platformLayer);
            if (co2D != null) {
                //Vector3Int localPlace = (new Vector3Int((int)allCell[i].x, (int)allCell[i].y, (int)tilemap.transform.position.y));
                //TileBase tile = tilemap.GetTile(localPlace);
                MapTiles.Add(allCell[i], null);
                TilesPos.Add(allCell[i]);
            }
            else {

            }
        }
    }
    void CreateNode()
    {
        //BoundsInt bounds = tilemap.cellBounds;
        //TileBase[] allTiles = tilemap.GetTilesBlock(bounds);
        Dictionary<Vector2, Node> NodeDic = new Dictionary<Vector2, Node>();

        for (int i = 0; i < TilesPos.Count; i++)
        {
            Vector2 tilePos = TilesPos[i];
            if (MapTiles.ContainsKey(tilePos))
            {
                int SurroundingTIles = 0;

                for (int x = -1; x <= 1; x++)
                {
                    for (int y = 0; y <= 1; y++)
                    {
                        if ((x == 0 || y == 0) && x != y)
                        {
                            Vector2 searchTilePos = new Vector2(tilePos.x + x, tilePos.y + y);
                            if (MapTiles.ContainsKey(searchTilePos))
                            {
                                SurroundingTIles++;
                                if (x == 0 && y == 1)
                                {
                                    SurroundingTIles += 100;
                                }
                            }
                        }
                    }
                }
                if (SurroundingTIles <= 1)
                {
                    Node node = new Node(tilePos);
                    NodeDic.Add(node.Pos, node);
                    NodeList.Add(node);
                }
            }
        }
        // 드롭 다운 포인트 설정

        for (int i = 0; i < NodeList.Count; i++)
        {
            Vector2 tilePos = NodeList[i].Pos;

            // 왼쪽 오른 쪽 대각선에 타일이 있는지 검사
            Vector2 diagonalTile = new Vector2(tilePos.x - 1, tilePos.y + 1);
            if (MapTiles.ContainsKey(diagonalTile))
                continue;

            diagonalTile = new Vector2(tilePos.x + 1, tilePos.y + 1);
            if (MapTiles.ContainsKey(diagonalTile))
                continue;

            for (int x = -1; x <= 1; x += 2)
            {
                Vector2 searchTilePos = new Vector2(tilePos.x + x, tilePos.y);
                if (!MapTiles.ContainsKey(searchTilePos))
                {
                    for (float y = searchTilePos.y; y > boxZone.GetLeftDownPoint().y; y--)
                    {
                        Vector2 dropDownTilePos = new Vector2(searchTilePos.x, y);
                        if (MapTiles.ContainsKey(dropDownTilePos))
                        {
                            Node node = new Node(dropDownTilePos);
                            bool AlreadyExist = false;
                            for (int j = 0; j < NodeList.Count; j++)
                            {
                                if (NodeList[j].Pos == node.Pos)
                                {
                                    node = NodeList[j];
                                    AlreadyExist = true;
                                    break;
                                }
                            }
                            if (!AlreadyExist) {
                                NodeList.Add(node);
                            }

                            ConnectNodes(NodeList[i], node, Node.ConnectNodeType.Down);

                            break;
                        }
                    }
                }
                
            }
        }

        // 오른 쪽 직선 노드 검색
        for (int i = 0; i < NodeList.Count; i++)
        {
            float DIstance = float.MaxValue;
            Node CloseNode = null;
            Vector2 rightTIle = new Vector2(NodeList[i].Pos.x + 1, NodeList[i].Pos.y);
            Vector2 diagonalTile = new Vector2(NodeList[i].Pos.x + 1, NodeList[i].Pos.y + 1);
            if (MapTiles.ContainsKey(rightTIle) && !MapTiles.ContainsKey(diagonalTile))
            {
                for (int k = 0; k < NodeList.Count; k++)
                {
                    if ((NodeList[i].Pos.y == NodeList[k].Pos.y) && (NodeList[i].Pos.x != NodeList[k].Pos.x))
                    {
                        if (NodeList[i].Pos.x < NodeList[k].Pos.x)
                        {
                            float distance = Mathf.Abs(NodeList[k].Pos.x - NodeList[i].Pos.x);
                            if (distance < DIstance)
                            {
                                DIstance = distance;
                                CloseNode = NodeList[k];
                            }
                        }
                    }
                }
            }
            if (CloseNode != null) {
                ConnectNodes(NodeList[i], CloseNode, Node.ConnectNodeType.Straight);
            }
        }

        // 오른 쪽 낭떠러지 노드 검색
        for (int i = 0; i < NodeList.Count; i++) {
            float DIstance = float.MaxValue;
            Node CloseNode = null;
            Vector2 rightTIle = new Vector2(NodeList[i].Pos.x + 1, NodeList[i].Pos.y);
            Vector2 diagonalTile = new Vector2(NodeList[i].Pos.x + 1, NodeList[i].Pos.y + 1);
            if (!MapTiles.ContainsKey(rightTIle) && !MapTiles.ContainsKey(diagonalTile)) {
                for (int k = 0; k < NodeList.Count; k++) {
                    if ((NodeList[i].Pos.y == NodeList[k].Pos.y) && (NodeList[i].Pos.x != NodeList[k].Pos.x) && Mathf.Abs(NodeList[i].Pos.x - NodeList[k].Pos.x) <= CliffDst) {
                        if (NodeList[i].Pos.x < NodeList[k].Pos.x) {
                            float distance = Mathf.Abs(NodeList[k].Pos.x - NodeList[i].Pos.x);
                            if (distance < DIstance) {
                                DIstance = distance;
                                CloseNode = NodeList[k];
                            }
                        }
                    }
                }
            }
            if (CloseNode != null) {
                ConnectNodes(NodeList[i], CloseNode, Node.ConnectNodeType.Cliff);
            }
        }

        //float Mass = TestBall.mass;
        float gravity = Gravity * 9.8f;
        int platformLayer = 1 << LayerMask.NameToLayer("Platform"); ;
        // 대각선 노드 검색
        for (int i = 0; i < NodeList.Count; i++) {
            float DIstance = float.MaxValue;
            Node CloseNode = null;
            Vector2 rightTIle = new Vector2(NodeList[i].Pos.x + 1, NodeList[i].Pos.y);
            Vector2 diagonalTile = new Vector2(NodeList[i].Pos.x + 1, NodeList[i].Pos.y + 1);

            for (int k = 0; k < NodeList.Count; k++) {
                Node underNode = NodeList[i].Pos.y < NodeList[k].Pos.y ? NodeList[i] : NodeList[k];
                bool next = false;
                // 위를 가로막는 블럭이 있을경우
                for (int blockYNode = 1; blockYNode < Mathf.Abs(NodeList[i].Pos.y - NodeList[k].Pos.y) + 1; blockYNode++) {
                    Vector2 searchPos = underNode.Pos + Vector2.up + Vector2.up * blockYNode;
                    if (MapTiles.ContainsKey(searchPos)) {
                        next = true;
                        break;
                    }
                }
                if (next)
                    continue;
                //Vector2 dir = NodeList[k].Pos - NodeList[i].Pos;
                //dir.Normalize();
                //LayerMask mask = LayerMask.NameToLayer("Platform");
                //RaycastHit2D[] ray = Physics2D.RaycastAll(NodeList[i].Pos, dir, Vector2.Distance(NodeList[i].Pos, NodeList[k].Pos), mask);
                //if (ray.Length >= 3)
                //    continue;
                float XDistance = Mathf.Abs(NodeList[i].Pos.x - NodeList[k].Pos.x);
                float YDistance = Mathf.Abs(NodeList[i].Pos.y - NodeList[k].Pos.y);
                if (XDistance >= 2 && YDistance >= 1) {
                    //float MaxX = Mathf.Sin(2 * Mathf.PI / 180) / Gravity;
                    float MaxY = (JumpPoawer * JumpPoawer * Mathf.Sin(90 * Mathf.PI / 180)) / (2 * gravity);
                    if (XDistance <= 5 && YDistance < MaxY) {
                        float distance = Mathf.Abs(NodeList[k].Pos.x - NodeList[i].Pos.x);
                        if (distance < DIstance) {

                            // 위로 향할 경우
                            if (NodeList[i].Pos.y < NodeList[k].Pos.y) {
                                float blockNodeDir = NodeList[i].Pos.x < NodeList[k].Pos.x ? -1 : 1;
                                Vector2 blockNodeSearch = NodeList[k].Pos + Vector2.right * blockNodeDir;
                                // 이동에 가로막는 블록이 없을경우
                                if (!MapTiles.ContainsKey(blockNodeSearch)) {
                                    DIstance = distance;
                                    CloseNode = NodeList[k];
                                }
                            }
                            else {
                                float blockNodeDir = NodeList[i].Pos.x < NodeList[k].Pos.x ? 1 : -1;
                                Vector2 blockNodeSearch = NodeList[i].Pos + Vector2.right * blockNodeDir;
                                // 이동에 가로막는 블록이 없을경우
                                if (!MapTiles.ContainsKey(blockNodeSearch)) {
                                    DIstance = distance;
                                    CloseNode = NodeList[k];
                                }
                            }

                        }
                    }
                }
            }

            if (CloseNode != null) {
                if (NodeList[i].Pos.y < CloseNode.Pos.y)
                    ConnectNodes(NodeList[i], CloseNode, Node.ConnectNodeType.Up);
                else
                    ConnectNodes(NodeList[i], CloseNode, Node.ConnectNodeType.Down);
            }
        }
        for (int i = 0; i < NodeList.Count; i++) {
            NodeList[i].Pos.y += 1;
        }
    }
  
    void ConnectNodes(Node Node1, Node Node2, Node.ConnectNodeType type)
    {
        float distance = Vector2.Distance(Node1.Pos, Node2.Pos);
        if (!Node1.NodeContains(Node2)) {
            Node1.ConnectNodesList.Add(new Node.ConnectNode(Node2, distance, type));
        }
        if (!Node2.NodeContains(Node1)) {
            if (type == Node.ConnectNodeType.Up) {
                Node2.ConnectNodesList.Add(new Node.ConnectNode(Node1, distance, Node.ConnectNodeType.Down));
            }
            else if (type == Node.ConnectNodeType.Down) {
                Node2.ConnectNodesList.Add(new Node.ConnectNode(Node1, distance, Node.ConnectNodeType.Up));
            }
            else {
                Node2.ConnectNodesList.Add(new Node.ConnectNode(Node1, distance, type));
            }
        }
    }

    public Node.ConnectNode[] StartPathfinder(Node StartNode, Node EndNode, float gravity, float jumpPower, float velocity)
    {
        Dictionary<Vector2, Path> PathDic = new Dictionary<Vector2, Path>();
        for (int i = 0; i< NodeList.Count; i ++)
        {
            Path path = new Path();
            path.node = NodeList[i];
            PathDic.Add(NodeList[i].Pos, path);
        }
        List<Path> CloseList = new List<Path>();
        List<Path> OpenList = new List<Path>();
        List<Node.ConnectNode> PathList = new List<Node.ConnectNode>();
        Path NowPath = new Path();
        Path NextPath = new Path();
        float MinF = float.MaxValue;
        NowPath.node = StartNode;
        int num = 0;
        float MaxY = (jumpPower * jumpPower * Mathf.Sin(90 * Mathf.PI / 180)) / (2 * gravity * 9.8f);
        while (NowPath.node.Pos != EndNode.Pos)
        {
            if (!CloseList.Contains(NowPath))
            {
                //CloseList.Add(NowPath);
                if (OpenList.Contains(NowPath))
                    OpenList.Remove(NowPath);
                for (int i = 0; i < NowPath.node.ConnectNodesList.Count; i++)
                {
                    if (NowPath.node != StartNode)
                    if (NowPath.node.ConnectNodesList[i].node == NowPath.ParentPath.node)
                        continue;
                    if (CloseList.Contains(PathDic[NowPath.node.ConnectNodesList[i].node.Pos]))
                        continue;
                    Path chiledPath = PathDic[NowPath.node.ConnectNodesList[i].node.Pos];
                    chiledPath.G = Vector2.Distance(chiledPath.node.Pos, NowPath.node.Pos) + NowPath.G;
                    chiledPath.H = Vector2.Distance(chiledPath.node.Pos, EndNode.Pos);
                    chiledPath.ParentPath = NowPath;
                    // 적당한 높이일 경우
                    if (NowPath.node.ConnectNodesList[i].node.Pos.y > NowPath.node.Pos.y) {
                        //Debug.Log("더 높다");
                        //Debug.Log("최대 높이 : " + MaxY);
                        float heightDifferent = NowPath.node.ConnectNodesList[i].node.Pos.y - NowPath.node.Pos.y;
                        if (MaxY <= heightDifferent) {
                            continue;
                        }
                        if (NowPath.node.ConnectNodesList[i].nodeType == Node.ConnectNodeType.Up) {
                            float xDifferent = Mathf.Abs(NowPath.node.ConnectNodesList[i].node.Pos.x - NowPath.node.Pos.x);
                            Vector2 dir = (NowPath.node.ConnectNodesList[i].node.Pos - NowPath.node.Pos).normalized;
                            float angle = (float)Math.Atan2(dir.y, dir.x);


                            float expectationJumpPower = Mathf.Sqrt((9.8f * 2 * xDifferent) / Mathf.Sin(90));
                            float scala = Mathf.Sqrt(expectationJumpPower * expectationJumpPower + velocity * velocity);
                            float maxXDifferent = (2 * (scala * scala) * Mathf.Sin(angle) * Mathf.Cos(angle)) / (9.8f * gravity);
                            maxXDifferent = Mathf.Abs(maxXDifferent);
                            if (xDifferent > maxXDifferent ) {
                                continue;
                            }
                        }
                    }
                    if (MinF > chiledPath.F)
                        NextPath = chiledPath;
                    if (!OpenList.Contains(chiledPath))
                        OpenList.Add(chiledPath);
                }
                CloseList.Add(NowPath);
                OpenList.Remove(NowPath);
                if (MinF != float.MaxValue)
                    NowPath = NextPath;
                MinF = float.MaxValue;
            }
            else
            {
                //Debug.Log("오픈 리스트 갯수 : " + OpenList.Count);
                //Debug.Log("클로즈 리스트 갯수 : " + CloseList.Count);
                if (OpenList.Count > 0)
                {
                    float openListMinF = float.MaxValue;
                    for(int i = 0;i < OpenList.Count; i++)
                    {
                        if (openListMinF > OpenList[i].F)
                        {
                            openListMinF = OpenList[i].F;
                            NowPath = OpenList[i];
                        }
                    }
                    OpenList.Remove(NowPath);
                    //Debug.Log(OpenList.Contains(NowPath) + " / " +CloseList.Contains(NowPath));
                }
                else
                    break;
            }
            num++;
            if (num > 10000)
                break;
        }
        if(NowPath.node.Pos == EndNode.Pos)
        {
            Path path;
            path = NowPath;
            while (path.node.Pos != StartNode.Pos)
            {
                Node.ConnectNode connectNode = path.ParentPath.node.FindConnectNode(path.node);
                Node.ConnectNode pathNode;
                float distance = Vector2.Distance(path.node.Pos, connectNode.node.Pos);
                if (connectNode.node != null)
                    pathNode = new Node.ConnectNode(path.node, distance, connectNode.nodeType);
                else {
                    // 시작 노드
                    pathNode = new Node.ConnectNode(path.node, distance, Node.ConnectNodeType.Straight);
                }
                PathList.Add(pathNode);
                path = path.ParentPath;
                //Debug.Log(path.node.Pos);
                num++;
                if (num > 10000)
                    break;
                if (path.ParentPath == null)
                {
                    break;
                }
            }
            Node.ConnectNode startConnectPathNode = new Node.ConnectNode(StartNode,0, Node.ConnectNodeType.Straight);
            PathList.Add(startConnectPathNode);
        }
        List<Node.ConnectNode> pathlist = new List<Node.ConnectNode>();
        for(int i = PathList.Count - 1; i >= 0; i--)
        {
            pathlist.Add(PathList[i]);
        }
        return pathlist.ToArray();
    }
    class Path
    {
        public Node node;
        public float F
        {
            get { return H + G; }
        }
        public float H;
        public float G;
        public Path ParentPath;
    }
    public Node GetCloseNode(Vector2 Pos)
    {
        float Distance = float.MaxValue;
        Node CloseNode = null;
        for (int i = 0; i < NodeList.Count; i++)
        {
            float distance = Vector2.Distance(Pos, NodeList[i].Pos);
            if(Distance > distance)
            {
                Vector2 dir = (NodeList[i].Pos - Pos).normalized;
                if (!Physics2D.Raycast(Pos, dir, distance, 1 << LayerMask.NameToLayer("Platform"))) {
                    CloseNode = NodeList[i];
                    Distance = distance;
                }
                //CloseNode = NodeList[i];
                //Distance = distance;
            }
        }
        return CloseNode;
    }
    private void OnDrawGizmos()
    {
        for(int i = 0; i < NodeList.Count; i++)
        {
            Vector2 NodePos = new Vector3(NodeList[i].Pos.x, NodeList[i].Pos.y);
            Gizmos.DrawWireSphere(NodePos, 0.3f);
            for (int k = 0; k < NodeList[i].ConnectNodesList.Count; k++)
            {
                Vector2 ConnnectNodePos = new Vector2(NodeList[i].ConnectNodesList[k].node.Pos.x, NodeList[i].ConnectNodesList[k].node.Pos.y);
                Gizmos.DrawLine(NodePos, ConnnectNodePos);
            }
        }       
    }
    private void OnGUI() {
    }
}
