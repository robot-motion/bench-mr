---
layout: default
title:  "Moving AI Planners"
date:   2021-01-04 13:20:59 +0100
parent: "Results"
nav_order: 1
---

# Moving AI

**Path planning** results from the [2D Pathfinding "MovingAI" Datasets](https://www.movingai.com/benchmarks/grids.html)
{: .fs-6 .fw-300 }

<link rel="stylesheet" href="https://stackpath.bootstrapcdn.com/bootstrap/4.3.1/css/bootstrap.min.css" integrity="sha384-ggOyR0iXCbMQv3Xipma34MD+dH/1fQ784/j6cY/iJTQUOhcWr7x9JvoRxT2MZw1T" crossorigin="anonymous">
<link rel="stylesheet" type="text/css" href="{{ site.baseurl }}/assets/results/home.css">
<link href="https://unpkg.com/tabulator-tables@4.5.3/dist/css/tabulator.min.css" rel="stylesheet">
<script type="text/javascript" src="https://unpkg.com/tabulator-tables@4.5.3/dist/js/tabulator.min.js"></script>

<script>    
var myData;
var stdData;
var planner_count = 0;
var filled = false;
var scenario_list = [];

var tabledata = [];
var table;
var max_sol = 0;
var max_t = 0;
var max_pl = 0;
var max_curv = 0;
var max_clear = 0;
var max_cusps = 0;
        
function loadJSON(callback, fPath) {  
    console.log('In loadJSON: ' + fPath); 
    var xmlr = new XMLHttpRequest();
    xmlr.overrideMimeType("application/json");
    xmlr.open('GET', fPath, false);
    xmlr.onreadystatechange = function () {
        if (xmlr.readyState == 4 && xmlr.status == "200") {
            callback(xmlr.responseText);
        }
    };
    xmlr.send(null);  
}

function getData(scenarioName) {
    console.log('In getData: ' + scenarioName);
    loadJSON(function(response) {
        tabledata = JSON.parse(response);
    }, '{{ site.baseurl }}/assets/results/table_json/' + scenarioName + '_table.json');
    max_sol = tabledata[tabledata.length - 1]['max_sol'];
    max_t = tabledata[tabledata.length - 1]['max_t'];
    max_pl = tabledata[tabledata.length - 1]['max_pl'];
    max_curv = tabledata[tabledata.length - 1]['max_curv'];
    max_clear = tabledata[tabledata.length - 1]['max_clear'];
    max_cusps = tabledata[tabledata.length - 1]['max_cusps'];

    tabledata = tabledata.slice(0, tabledata.length-1);
    console.log('max_sol: ' + max_sol);
    console.log('json length: ' + tabledata.length);
    console.log('Finished loading: ' + tabledata);
}

function update_table() {
    document.getElementById("home_message").style.display = "none";
    document.getElementById("data_table_area").style.display = "block";
    var scenario_selection = document.getElementById("scenario_select");
    var selected_scenario_text = scenario_selection.options[scenario_selection.selectedIndex].text;
    getData(selected_scenario_text);
    document.getElementById("data_table_area").style.display = "block";
    table = new Tabulator("#data_table", {
        height:610,
        data:tabledata,
        layout:"fitColumns",
        columns:[
            {title:"Planner", field:"planner", align:"center", width:150},
            {title:"Solutions", field:"solutions", align:"left", sorter:"number", formatter:"progress", formatterParams:{
                min:0.0,
                max:max_sol,
                color:["#85e085"],
                legend: function(value) {
                    return (value + " / " + max_sol);
                },
                legendColor:"#000000",
                legendAlign:"center",
            }},
            {title:"Time [s]", field:"time", sorter:"number", align:"left",formatter:"progress", formatterParams:{
                min:0.0,
                max:max_t,
                color:function(value) {
                    if (value[0] >= '0' && value[0] <= '9') {
                        return "#ffbf80";
                    } else {
                        return "transparent";
                    }
                },
                legend: function(value) {
                    return value;
                },
                legendColor:"#000000",
                legendAlign:"center",
            }},
            {title:"Path Length", field:"path_length", sorter:"number", align:"left",formatter:"progress", formatterParams:{
                min:0.0,
                max:max_pl,
                color:function(value) {
                    if (value[0] >= '0' && value[0] <= '9') {
                        return "#ffbf80";
                    } else {
                        return "transparent";
                    }
                },
                legend: function(value) {
                    return value;
                },
                legendColor:"#000000",
                legendAlign:"center",
            }},
            {title:"Curvature", field:"curvature", sorter:"number", align:"left",formatter:"progress", formatterParams:{
                min:0.0,
                max:max_curv,
                color:function(value) {
                    if (value[0] >= '0' && value[0] <= '9') {
                        return "#ffbf80";
                    } else {
                        return "transparent";
                    }
                },
                legend: function(value) {
                    return value;
                },
                legendColor:"#000000",
                legendAlign:"center",
            }},
            {title:"Clearance", field:"clearance", sorter:"number", align:"left",formatter:"progress", formatterParams:{
                min:0.0,
                max:max_clear,
                color:function(value) {
                    if (value[0] >= '0' && value[0] <= '9') {
                        return "#ffbf80";
                    } else {
                        return "transparent";
                    }
                },
                legend: function(value) {
                    return value;
                },
                legendColor:"#000000",
                legendAlign:"center",
            }},
            {title:"Cusps", field:"cusps", sorter:"number", align:"left",formatter:"progress", formatterParams:{
                min:0.0,
                max:max_cusps,
                color:function(value) {
                    if (value[0] >= '0' && value[0] <= '9') {
                        return "#ffbf80";
                    } else {
                        return "transparent";
                    }
                },
                legend: function(value) {
                    return value;
                },
                legendColor:"#000000",
                legendAlign:"center",
            }},
        ],
        rowClick:function(e, row){
            alert("Row " + row.getData().id + " Clicked!!!!");
        },
    });
    table.redraw(true);
}

function show_graph(selected_graph){
    document.getElementById("plots_area").innerHTML = "";
    console.log(selected_graph);
    if(selected_graph != "None"){
        document.getElementById("plots_area").innerHTML = "<a href='" + selected_graph + ".png" + "'><img src='" + selected_graph + ".png" + "' style='width: 100%'></a>";
    }
}

function get_result(){
    document.getElementById("home_message").style.display = "none";
    var scenario_selection = document.getElementById("scenario_select");
    var steer_selection = document.getElementById("steer_select");
    var scenario_string = scenario_selection.options[scenario_selection.selectedIndex].text;
    var steer_string = steer_selection.options[steer_selection.selectedIndex].text;
    var selected_graph = "{{ site.baseurl }}/assets/results/img/" + scenario_string + "_" + steer_string + "_stats";
    show_graph(selected_graph);
    /* update_table(); */
}

function get_scenario_list() {
    var fPath = '{{ site.baseurl }}/assets/results/scenario_list.json';
    loadJSON(function(response) {
        scenario_list = JSON.parse(response);
        scenario_list.sort();
    }, fPath);
}
get_scenario_list();
</script>


<div id="content">
    <div id="select_box">
        <!-- select form -->
        <form>
            <div class="form-group row">
                <label for="overview_scenario_select" class="col-sm-3 col-form-label">Scenario</label>
                <div class="col-sm-8">
                    <select class="form-control" id="scenario_select">
                        <script>
                            var scenario_select = document.getElementById('scenario_select');
                            for(var i=0; i<scenario_list.length; i++){
                                scenario_select.innerHTML += '<option value="' + (i+1) + '">' + scenario_list[i] + '</option>';
                            }
                        </script>
                    </select>
                </div>
            </div>
            <div class="form-group row">
                <label for="overview_steer_select" class="col-sm-3 col-form-label">Steer Function</label>
                <div class="col-sm-8">
                    <select class="form-control" id="steer_select">
                        <option value="1">cc_reeds_shepp</option>
                        <option value="2">dubins</option>
                        <option value="3">posq</option>
                        <option value="4">reeds_shepp</option>
                    </select>
                </div>
            </div>
            <div class="row">
                <button type="button" onclick="get_result()" class="btn btn-primary" id="submit_button">Get Result</button>      
            </div>       
        </form>
    </div>
</div>
            
<div class="card" id="home_message">
    <div class="card-header" style="background-color:  #83888d; color: white; font-size: 20px;">
        Get Started
    </div>
    <div class="card-body" style="font-size: 20px;">
        <p>Please choose from the panel and click on "Get Result".</p>
    </div>
</div>

<div id="plots_area"></div>

<!-- <div id="data_table_area"><div id="data_table"></div></div> -->
        
<script src="https://code.jquery.com/jquery-3.3.1.slim.min.js" integrity="sha384-q8i/X+965DzO0rT7abK41JStQIAqVgRVzpbzo5smXKp4YfRvH+8abtTE1Pi6jizo" crossorigin="anonymous"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/popper.js/1.14.7/umd/popper.min.js" integrity="sha384-UO2eT0CpHqdSJQ6hJty5KVphtPhzWj9WO1clHTMGa3JDZwrnQq4sF86dIHNDz0W1" crossorigin="anonymous"></script>
<script src="https://stackpath.bootstrapcdn.com/bootstrap/4.3.1/js/bootstrap.min.js" integrity="sha384-JjSmVgyd0p3pXB1rRibZUAYoIIy6OrQ6VrjIEaFf/nJGzIxFDsf4x0xIM+B07jRM" crossorigin="anonymous"></script>
