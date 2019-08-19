
  // Function making the placement (left margin) of the workspace dynamic
  function placeWorkspaceLeft(){
    var windowWidth = window.innerWidth;
    var marginLeftWorkspace = (((windowWidth/2)-800)/2)+(((windowWidth/2)-800)/4);
    marginLeftWorkspace = Math.ceil(marginLeftWorkspace);
    return marginLeftWorkspace;
  }

  // Function making the placement (top margin) of the workspace dynamic
  function placeWorkspaceTop(){
    var windowHeight = window.innerHeight;
    console.log("height, enligt workspace: " + windowHeight);
    var marginTopWorkspace = (windowHeight-800)*0.35;
    // var marginLeftSimulation = 3*(((windowWidth/2)-800)/2)+800;
    marginTopWorkspace = Math.ceil(marginTopWorkspace);
    console.log("marginTop, enligt workspace: " + marginTopWorkspace);
    return marginTopWorkspace;
  }

  function cancelSimulation(){
    cancel_simulation = true;
  }
