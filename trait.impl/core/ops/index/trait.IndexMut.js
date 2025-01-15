(function() {
    var implementors = Object.fromEntries([["heapless",[["impl&lt;'a, K, Q, V, S, const N: usize&gt; IndexMut&lt;&amp;'a Q&gt; for <a class=\"struct\" href=\"heapless/struct.IndexMap.html\" title=\"struct heapless::IndexMap\">IndexMap</a>&lt;K, V, S, N&gt;<div class=\"where\">where\n    K: Eq + Hash + Borrow&lt;Q&gt;,\n    Q: ?Sized + Eq + Hash,\n    S: BuildHasher,</div>"],["impl&lt;'a, K, V, Q, const N: usize&gt; IndexMut&lt;&amp;'a Q&gt; for <a class=\"struct\" href=\"heapless/struct.LinearMap.html\" title=\"struct heapless::LinearMap\">LinearMap</a>&lt;K, V, N&gt;<div class=\"where\">where\n    K: Borrow&lt;Q&gt; + Eq,\n    Q: Eq + ?Sized,</div>"]]]]);
    if (window.register_implementors) {
        window.register_implementors(implementors);
    } else {
        window.pending_implementors = implementors;
    }
})()
//{"start":57,"fragment_lengths":[618]}