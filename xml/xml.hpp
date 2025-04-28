inline bool Xml::Open()
{
    /*
     * this initialize the library and check potential ABI mismatches
     * between the version it was compiled for and the actual shared
     * library used.
     */
    LIBXML_TEST_VERSION

    /* parse the file and get the DOM */
    std::string xml_file = getParamStr("file_name");
    doc = xmlReadFile(xml_file.c_str(), NULL, 0);

    if (doc == NULL)
    {
        printf("error: could not parse file %s\n", xml_file.c_str());
        return false;
    }

    /* get the root element node */
    root_element = xmlDocGetRootElement(doc);
    if (root_element == NULL)
    {
        std::cerr << "Empty XML file: " << xml_file << std::endl;
        xmlFreeDoc(doc);
        return false;
    }
    return true;
    /* parse all the element nodes */
    // get_nodes_in_xml(root_element);
    // print_configs();

    /* free the document */
    
}
inline bool Xml::Close()
{
    xmlFreeDoc(doc);

    /*
     * Free the global variables that may
     * have been allocated by the parser.
     */
    xmlCleanupParser();
    return true;
}
inline bool Xml::Read()
{
    if(!Open())
        return false;

    xmlNode *cur_node = NULL;
    std::string root_name = getParamStr("root");
    for (cur_node = root_element; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if(!readChildren("",cur_node->children))
                return false;
        }
    }
    if(!Close())
        return false;
    return true;
}

inline bool Xml::Write()
{
    doc = xmlNewDoc(BAD_CAST "1.0");

    root_element  = xmlNewNode(NULL, BAD_CAST getParamStr("root").c_str());
    xmlDocSetRootElement(doc, root_element);
    params = getAllParams();
    for(auto iter = params.begin(); iter!=params.end(); ++iter)
    {
        if(iter->first == "root" || iter->first == "file_name")
            continue;
        
        xmlNode *node = xmlNewNode(NULL, BAD_CAST iter->first.c_str());
        xmlNode *data = xmlNewText(BAD_CAST getParamStr(iter->first).c_str());
        xmlAddChild(node,data);
        xmlAddChild(root_element, node);
    }
    int ret = xmlSaveFormatFileEnc(getParamStr("file_name").c_str(), doc, "UTF-8", 1);
    if (ret == -1) {
        std::cout << "Failed to write XML file " << getParamStr("file_name") <<"\n";
        return false;
    }
    if(!Close())
        return false;
    return true;
}

inline bool Xml::readChildren(std::string name, xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next) 
    {
        if (cur_node->type == XML_ELEMENT_NODE) 
        {
            std::string key((char *)cur_node->name);

            // Check if the node has no element children
            bool hasChildren = false;
            int cnt = getParamInt("target_cnt");
            for (xmlNode *child = cur_node->children; child; child = child->next) 
            {
                if (child->type == XML_ELEMENT_NODE) 
                {
                    hasChildren = true;
                    if(strcmp((char *)node->parent->name, "target") == 0)
                    {
                        insertParam("target" + std::to_string(cnt), "target_"+key+"_");
                        insertParam("target_cnt", ++cnt);
                    }
                    break;
                }
            }

            if (!hasChildren) 
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                std::string value((char *)s);
                if ((strcmp((char *)s, "") == 0) ||
                    (strcmp((char *)s, "NULL") == 0) ||
                    (strcmp((char *)s, " ") == 0) ||
                    (strcmp((char *)s, "null") == 0)) 
                {
                    value = "";
                }

                // if (name == "") 
                //     name += "_";
                insertParam(name + key, value);
                xmlFree(s);
            } 
            else 
            {
                if (cur_node->children)
                    readChildren(name + key + "_", cur_node->children);
            }
        }
    }
    return true;
}