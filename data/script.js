function onButton() {
    var xhttp = new XMLHttpRequest();
    xhttp.open("GET", "on", true);
    xhttp.send();
}

function offButton() {
    var xhttp = new XMLHttpRequest();
    xhttp.open("GET", "off", true);
    xhttp.send();
}

setInterval(function getData()
{
    var xhttp = new XMLHttpRequest();

    xhttp.onreadystatechange = function()
    {
        if(this.readyState == 4 && this.status == 200)
        {
            document.getElementById("valeurTemperature").innerHTML = this.responseText;
        }
    };

    xhttp.open("GET", "lireTemperature", true);
    xhttp.send();
}, 2000);

setInterval(function getData()
{
    var xhttp = new XMLHttpRequest();

    xhttp.onreadystatechange = function()
    {
        if(this.readyState == 4 && this.status == 200)
        {
            document.getElementById("valeurStop").innerHTML = this.responseText;
        }
    };

    xhttp.open("GET", "lireStop", true);
    xhttp.send();
}, 2000);

setInterval(function getData()
{
    var xhttp = new XMLHttpRequest();

    xhttp.onreadystatechange = function()
    {
        if(this.readyState == 4 && this.status == 200)
        {
            document.getElementById("valeurClignoG").innerHTML = this.responseText;
        }
    };

    xhttp.open("GET", "lireClignoG", true);
    xhttp.send();
}, 2000);

setInterval(function getData()
{
    var xhttp = new XMLHttpRequest();

    xhttp.onreadystatechange = function()
    {
        if(this.readyState == 4 && this.status == 200)
        {
            document.getElementById("valeurClignoD").innerHTML = this.responseText;
        }
    };

    xhttp.open("GET", "lireClignoD", true);
    xhttp.send();
}, 2000);

setInterval(function getData()
{
    var xhttp = new XMLHttpRequest();

    xhttp.onreadystatechange = function()
    {
        if(this.readyState == 4 && this.status == 200)
        {
            document.getElementById("valeurHumidite").innerHTML = this.responseText;
        }
    };

    xhttp.open("GET", "lireHumidite", true);
    xhttp.send();
}, 2000);

setInterval(function getData()
{
    var xhttp = new XMLHttpRequest();

    xhttp.onreadystatechange = function()
    {
        if(this.readyState == 4 && this.status == 200)
        {
            document.getElementById("valeurPression").innerHTML = this.responseText;
        }
    };

    xhttp.open("GET", "lirePression", true);
    xhttp.send();
}, 2000);