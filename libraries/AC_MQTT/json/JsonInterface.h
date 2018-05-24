/*
 * JsonInterface.h
 *
 *  Created on: 2017-12-29
 *      Author: liwh1
 */

#pragma once

#include "cJSON.h"

class JSONInterface
{
public:

    // constructor
	JSONInterface(){
		root = cJSON_CreateArray();
		dir  = cJSON_CreateObject();

		cJSON_AddItemToArray(root, dir);
	     //为对象添加字符串键值对

	     cJSON_AddStringToObject(dir,"key","123");
	     cJSON_AddStringToObject(dir,"Gps1","1");
	     cJSON_AddStringToObject(dir,"Gps2","2");
	     cJSON_AddStringToObject(dir,"Gps3","3");
	     cJSON_AddStringToObject(dir,"Gps4","4");
	     cJSON_AddStringToObject(dir,"Gps5","5");


	}

	void creat(){
		cJSON_ReplaceItemInObject(dir, "Gps1", cJSON_CreateString("hello"));
	}

	char *getOutPut(){
		return cJSON_Print(root);
	}

private:

	cJSON *root;
	cJSON *dir;
};
/*
 *     cJSON *root,*dir1,*dir2,*dir3;

     char *out;

     //创建json数组型结构体

     root = cJSON_CreateArray();

     //为数组添加对象

     cJSON_AddItemToArray(root,dir1=cJSON_CreateObject());

     //为对象添加字符串键值对

     cJSON_AddStringToObject(dir1,"name",".");

     cJSON_AddStringToObject(dir1,"path","uploads/");

     cJSON_AddStringToObject(dir1,"flag","true");

     cJSON_AddItemToArray(root,dir2=cJSON_CreateObject());

     cJSON_AddStringToObject(dir2,"name","..");

     cJSON_AddStringToObject(dir2,"path","uploads");

     cJSON_AddStringToObject(dir2,"flag","true");

     cJSON_AddItemToArray(root,dir3=cJSON_CreateObject());

     cJSON_AddStringToObject(dir3,"name","wang.txt");

     cJSON_AddStringToObject(dir3,"path","uploads/wang.txt");

     cJSON_AddStringToObject(dir3,"flag","false");

     //将json结构体转换为字符串

     out=cJSON_Print(root);

     //删除

     cJSON_Delete(root);

     return out;
     */
