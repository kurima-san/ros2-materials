
document.querySelectorAll('pre').forEach(pre=>{
 const b=document.createElement('button'); b.className='copy'; b.textContent='Copy';
 b.onclick=async()=>{const c=pre.querySelector('code'); await navigator.clipboard.writeText(c?c.innerText:pre.innerText);
 b.textContent='Copied'; setTimeout(()=>b.textContent='Copy',900);}; pre.appendChild(b);
});
