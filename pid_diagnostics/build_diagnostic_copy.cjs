// Creates a passive diagnostic MDL copy. Does not start MATLAB or hardware.
const fs = require('node:fs');
const path = require('node:path');
const project = path.resolve(__dirname, '..');
const source = 'Stabilizer_QDrone_048378';
const target = source + '_PID_Diag';
const input = fs.readFileSync(path.join(project, source + '.mdl'), 'latin1');
let text = input.replaceAll(source, target);
let sid = 13000;
function parse() {
  const root = {type:'root', children:[], props:{}};
  const stack = [root]; let offset=0;
  for (const line of text.split(/(?<=\n)/)) {
    const trimmed=line.trim(); const open=trimmed.match(/^(\S+)\s*\{$/);
    if(open) {
      const node={type:open[1],start:offset,children:[],props:{},parent:stack.at(-1)};
      stack.at(-1).children.push(node); stack.push(node);
    } else if(trimmed==='}') {
      const node=stack.pop(); node.close=offset; node.end=offset+line.length;
    } else {
      const property=trimmed.match(/^(\S+)\s+(.+)$/);
      if(property)stack.at(-1).props[property[1]]=property[2];
    }
    offset+=line.length;
  }
  if(stack.length!==1)throw Error('Unbalanced MDL');
  return root;
}
function all(node,type) {return [...(node.type===type?[node]:[]),...node.children.flatMap(n=>all(n,type))];}
function system(name) {
  const found=all(parse(),'System').filter(n=>n.props.Name===JSON.stringify(name));
  if(found.length!==1)throw Error('Expected unique subsystem: '+name);
  return found[0];
}
function block(sys,name){const found=sys.children.filter(n=>n.type==='Block'&&n.props.Name===JSON.stringify(name));if(found.length!==1)throw Error('Missing/ambiguous block '+name);return found[0];}
function append(sysName,body){
  const s=system(sysName);
  // MDL loading resolves destinations as lines are read. Define new blocks
  // before the first existing line that can refer to them.
  const firstLine=s.children.find(n=>n.type==='Line');
  const at=firstLine?firstLine.start:s.close;
  text=text.slice(0,at)+body+text.slice(at);
}
function tap(sysName,name,port,dest,destPort){
  const s=system(sysName); block(s,name);
  const lines=s.children.filter(n=>n.type==='Line'&&n.props.SrcBlock===JSON.stringify(name)&&n.props.SrcPort===String(port));
  if(lines.length!==1)throw Error('Expected one source line: '+sysName+'/'+name);
  const line=lines[0];let raw=text.slice(line.start,line.end);
  if(line.props.DstBlock){
    // Preserve the existing destination as a branch before adding our new sink.
    raw=raw.replace(/^\s*DstBlock\s+[^\r\n]+\r?\n/m,'').replace(/^\s*DstPort\s+[^\r\n]+\r?\n/m,'');
    raw=raw.replace(/\s*}\s*$/, '\n Branch {\n DstBlock '+line.props.DstBlock+'\n DstPort '+line.props.DstPort+'\n }\n}\n');
  }
  raw=raw.replace(/\s*}\s*$/, '\n Branch {\n DstBlock '+JSON.stringify(dest)+'\n DstPort '+destPort+'\n }\n}\n');
  text=text.slice(0,line.start)+raw+text.slice(line.end);
}
function basic(type,name,props){return '\n Block {\n BlockType '+type+'\n Name '+JSON.stringify(name)+'\n SID "'+(++sid)+'"\n '+props+'\n }\n';}
function instrument(sysName,prefix,signals){
  const s=system(sysName);const template=block(s,'To Host File');
  let logger=text.slice(template.start,template.end);
  const replacements={Name:JSON.stringify('PID Diagnostic Log'),SID:'"'+(++sid)+'"',Position:'[1650 1180 1790 1240]',file_name:JSON.stringify('saved_data/%m/'+prefix+'_%{char(datetime)}.mat'),variable_name:JSON.stringify('pid_'+prefix.toLowerCase()),decimation:'"2"',final_file_name:JSON.stringify('saved_data/'+target+'/'+prefix+'_pending.mat')};
  for(const [key,value] of Object.entries(replacements))logger=logger.replace(new RegExp('^([ \\t]*)'+key+'[ \\t]+[^\\r\\n]+','m'),(_,indent)=>indent+key+' '+value);
  let blocks=basic('Clock','PID Diagnostic Time','Position [1330 1100 1370 1120]');
  blocks+=basic('Mux','PID Diagnostic Signals','Ports ['+(signals.length+1)+', 1]\n Inputs "'+(signals.length+1)+'"\n Position [1550 1120 1555 1420]');
  blocks+=logger;
  blocks+=basic('Scope','PID Diagnostic Scope','Ports [1]\n Position [1650 1300 1690 1340]\n NumInputPorts "1"\n Floating off');
  blocks+=' Line {\n SrcBlock "PID Diagnostic Time"\n SrcPort 1\n DstBlock "PID Diagnostic Signals"\n DstPort 1\n }\n';
  blocks+=' Line {\n SrcBlock "PID Diagnostic Signals"\n SrcPort 1\n DstBlock "PID Diagnostic Log"\n DstPort 1\n }\n';
  append(sysName,blocks);
  signals.forEach((signal,i)=>tap(sysName,signal,1,'PID Diagnostic Signals',i+2));
  tap(sysName,prefix==='Motors'?'Maximum \nCommand\nAuthority ':'Sum3',1,'PID Diagnostic Scope',1);
}
instrument('STABILIZER - QDRONE','Controller',[
  'Saturate\nStabilization\nCommand','Select Angular Position','Select Angular\nSpeed',
  'Sum1','Sum3','Saturation\nCommand \nAuthority (Nm)','Throttle (N)','Arm (0,1)','Mode (0,1)'
]);
instrument('PHYSICAL PLANT w/ LINEAR THRUST','Motors',[
  'Product','Maximum \nCommand\nAuthority ','Bias - Pulse \nOffset (%)','Conversion','Arm (0,1)'
]);
text=text.replace(/SIDHighWatermark\s+"\d+"/,'SIDHighWatermark "'+sid+'"');
// Verify every old block keeps every original property, except model-name paths.
const newNodes=all(parse(),'Block');
const bySid=new Map(newNodes.map(n=>[n.props.SID,n]));
const output=text;text=input.replaceAll(source,target);
const oldNodes=all(parse(),'Block').filter(n=>n.props.SID);text=output;
for(const old of oldNodes){const updated=bySid.get(old.props.SID);if(!updated)throw Error('Removed original block');for(const [key,value] of Object.entries(old.props)){if(updated.props[key]!==value)throw Error('Changed original property '+old.props.Name+'/'+key);}}
function connections(tree){const result=[];for(const s of all(tree,'System')){const chain=[];for(let n=s;n;n=n.parent)if(n.type==='System')chain.unshift(n.props.Name);for(const l of s.children.filter(n=>n.type==='Line')){const visit=n=>{if(n.props.DstBlock)result.push(JSON.stringify([chain,l.props.SrcBlock,l.props.SrcPort,n.props.DstBlock,n.props.DstPort]));for(const c of n.children.filter(n=>n.type==='Branch'))visit(c);};visit(l);}}return result;}
const newEdges=new Set(connections(parse()));text=input.replaceAll(source,target);
const oldEdges=connections(parse());text=output;
for(const edge of oldEdges)if(!newEdges.has(edge))throw Error('Changed an original connection: '+edge);
const ids=newNodes.filter(n=>n.props.SID).map(n=>n.props.SID);
if(ids.length!==new Set(ids).size)throw Error('Duplicate SID');
for(const s of all(parse(),'System')){
  const defined=new Set();
  for(const n of s.children){
    if(n.type==='Block')defined.add(n.props.Name);
    if(n.type==='Line'){
      if(!defined.has(n.props.SrcBlock))throw Error('Source declared after line');
      for(const branch of [n,...all(n,'Branch')])if(branch.props.DstBlock&&!defined.has(branch.props.DstBlock))throw Error('Destination declared after line: '+branch.props.DstBlock);
    }
  }
}
const file=path.join(project,target+'.mdl');
if(fs.existsSync(file))fs.copyFileSync(file,file+'.'+Date.now()+'.bak');
fs.writeFileSync(file,text,'latin1');
fs.mkdirSync(path.join(project,'saved_data',target),{recursive:true});
console.log(JSON.stringify({file,originalBlocks:oldNodes.length,diagnosticBlocks:ids.length-oldNodes.length,originalConnectionsPreserved:oldEdges.length,sourceUnchanged:fs.readFileSync(path.join(project,source+'.mdl'),'latin1')===input}));
