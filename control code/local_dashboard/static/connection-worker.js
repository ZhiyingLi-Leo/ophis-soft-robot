// Successful server replies drive the next request; no background interval is needed.
let client='', renewMotors=false, running=false;
self.onmessage=({data})=>{
  client=data.client;renewMotors=!!data.renewMotors;
  if(!running&&client){running=true;keepalive();}
};
async function keepalive(){
  while(client){
    const abort=new AbortController();const timer=setTimeout(()=>abort.abort(),1000);
    let failed=false;
    try{
      const response=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({action:'heartbeat',client,wait:true,renew_motors:renewMotors}),signal:abort.signal});
      if(!response.ok)throw Error('HTTP '+response.status);
      self.postMessage(await response.json());
    }catch(error){failed=true;self.postMessage({ok:false,error:String(error)});}
    finally{clearTimeout(timer);}
    if(failed)await new Promise(resolve=>setTimeout(resolve,250));
  }
  running=false;
}
